#include "tool_util.hpp"
#include "sketch.hpp"
#include "ray.hpp"
#include "cluster.hpp"

#include <map>

/* Tool Management */

// Initialize all member variables
void ToolUtil::initTool() {
	// reset member variables
	resetSurfacePointsIndex();
	resetBasisPoints();
	resetSketchPoints();
	resetBasisConvexHull();
}

void ToolUtil::resetTool() {
	// reset mode enum
	*currentMode = 0;

	// update point clouds
	basisPointCloud = PointCloud(pointCloud->getShader(), basisPoints);
}

// Reset member variables (vector and set)
void ToolUtil::resetSurfacePointsIndex() {
	surfacePointsIndex.clear();
}
void ToolUtil::resetBasisPoints() {
	basisPoints.clear();
}
void ToolUtil::resetSketchPoints() {
	sketchPoints.clear();
}
void ToolUtil::resetBasisConvexHull() {
	basisConvexHull.clear();
}

/* Geometry Processing */

// Compute the surface points where mouse is currently hovering
// and then update surfacePointsIndex
//  - p: the screen coordinate [-1.0f, 1.0f]
//  - K_size: the selected nearest neighbors size
void ToolUtil::updateSurfacePoints(SketchPoint p, int K_size) {
	if (p.x < -1.0f || 1.0f < p.x) return;
	if (p.y < -1.0f || 1.0f < p.y) return;

	/* Cast a ray to the point cloud */
	Ray ray(camera, p.x, p.y);
	Ray::Hit hit = ray.rayMarching(pointCloud, calcCastedAverageDist());
	if (!hit.hit) return;

	/* Search the point cloud for nearest neighbors */
	glm::vec3 rayDir = hit.rayDir;
	glm::vec3 center = hit.pos;

	std::vector<int>		hitPointIndices;
	std::vector<float>	hitPointDistances;
	int hitPointCount = pointCloud->getOctree()->nearestKSearch(
		pcl::PointXYZ(center.x, center.y, center.z),
		K_size,
		hitPointIndices,
		hitPointDistances
	);

	std::vector<Vertex>* verticesPtr = pointCloud->getVertices();
	for (int i = 0; i < hitPointCount; i++) {
		int idx = hitPointIndices[i];
		Vertex v = (*verticesPtr)[idx];

		// If the normal does not face rayDir, then skip it.
		glm::vec3 pn = glm::vec3(v.nx, v.ny, v.nz);
		if (glm::dot(pn, rayDir) >= 0.0) continue;

		// Add nearest neigbor to surfacePointsIndex
		surfacePointsIndex.insert(idx);
	}
}

// Add the specified point to sketchPoints
void ToolUtil::addSketchPoint(SketchPoint p) {
	if (p.x < -1.0f || 1.0f < p.x) return;
	if (p.y < -1.0f || 1.0f < p.y) return;
	sketchPoints.emplace_back(p);
}

// Find basis points
//	1. Process ray-marching to find a candidate point for each grid
//	2. Detect depth with DBSCAN
//	
//	- addSurfacePoints:	if true, add surfacePoints to basisPoints
//	- clusteringMode:		clustering mode for DBSCAN
void ToolUtil::findBasisPoints(
	bool addSurfacePoints,
	int clusteringMode
) {	
	std::vector<Vertex>* verticesPtr = pointCloud->getVertices();
	const size_t verticesSize = verticesPtr->size();

	/* Initialize candidate points for basis */
	std::vector<Vertex> candidatePoints;
	std::set<int>				checkedPointsIndices;
	// Add the surface points to candidate points, if addSurfacePoints is true
	if (addSurfacePoints) {
		for (int idx: surfacePointsIndex) {
			candidatePoints.emplace_back((*verticesPtr)[idx]);
			checkedPointsIndices.insert(idx);
		}
	}

	/* Process ray-marching to find a candidate point for each grid */	
	// Compute the minimum rectangle that contains the sketch
	float min_x = 1e5f, max_x = -1e5f;
	float min_y = 1e5f, max_y = -1e5f;
	for (SketchPoint p: sketchPoints) {
		min_x = std::min(min_x, p.x);
		max_x = std::max(max_x, p.x);
		min_y = std::min(min_y, p.y);
		max_y = std::max(max_y, p.y);
	}

	assert(-1.0f <= min_x && min_x <= 1.0f);
	assert(-1.0f <= max_x && max_x <= 1.0f);
	assert(-1.0f <= min_y && min_y <= 1.0f);
	assert(-1.0f <= max_y && max_y <= 1.0f);

	// Cast rays for each grid in sketch
	float castedAverageDist = calcCastedAverageDist();
	float grid_x = (castedAverageDist/camera->getScreenWidth())*2.0f;
	float grid_y = (castedAverageDist/camera->getScreenWidth())*2.0f;

	int rayCount = 0;
	for (float x = min_x; x <= max_x; x += grid_x*2.0f) {
		for (float y = min_y; y <= max_y; y += grid_y*2.0f) {
			if (!insideSketch(SketchPoint{x, y})) continue;

			rayCount++;

			// Cast a ray
			Ray ray_pointcloud(camera, x, y);
			Ray::Hit hit_pointcloud = ray_pointcloud.rayMarching(pointCloud, castedAverageDist);
			if (!hit_pointcloud.hit) continue;
			
			// If the hit point is already checked, the skip it
			if (checkedPointsIndices.count(hit_pointcloud.index)) continue;
			checkedPointsIndices.insert(hit_pointcloud.index);

			// Project the hit point to the screen -> check if inside sketch
			Ray ray_screen(camera, hit_pointcloud.pos);
			Ray::Hit hit_screen = ray_screen.castPointToScreen();
			glm::vec2 pos_screen = camera->mapWorldToScreen(hit_screen.pos);
			if (!insideSketch(SketchPoint{pos_screen.x, pos_screen.y})) continue;

			// Add the hit point to candidatePoints
			glm::vec3 hit_p = hit_pointcloud.pos;
			glm::vec3 hit_n = hit_pointcloud.normal;
			candidatePoints.emplace_back(Vertex{
				hit_p.x, hit_p.y, hit_p.z,
				hit_n.x, hit_n.y, hit_n.z
			});
		}
	}

	/* Detect depth with DBSCAN */
	Clustering clustering(&candidatePoints, camera->getWorldDirection(), clusteringMode);
	basisPoints = clustering.executeClustering(pointCloud->getAverageDistance(), 1);

	std::cout << "Ray Count: " << rayCount << std::endl;
}

// Check whether (x, y) is inside or outside of the sketch.
//  1.  Draw a half-line parallel to the x-axis from a point.
//  2.  Determine that if there are an odd number of intersections
//      between this half-line and the polygon, it is inside, and if 
//      there are an even number, it is outside.
bool ToolUtil::insideSketch(SketchPoint p) {
	const int sketchPointsSize = sketchPoints.size();

	int crossCount = 0;
	for (int i = 0; i < sketchPointsSize; i++) {
		SketchPoint u = sketchPoints[i];
		SketchPoint v = sketchPoints[(i + 1)%sketchPointsSize];
		if (crossLines(p, u, v)) crossCount++;
	}

	if (crossCount%2 == 0) return false;
	else return true;
}

// Check whether (x, y) is inside or outside of the basisConvexHull.
//  1.  If haven't already calculated the convex hull of the basis points,
//      then calculate it.
//  2.  Draw a half-line parallel to the x-axis from a point.
//  3.  Determine that if there are an odd number of intersections
//      between this half-line and the polygon, it is inside, and if 
//      there are an even number, it is outside.
bool ToolUtil::insideBasisConvexHull(SketchPoint p) {
	if (basisPoints.size() < 3) return false;

	// If haven't already calculated the convex hull, then calculate it
	if (basisConvexHull.size() == 0) {
		computeBasisConvexHull();
	}

	// Check whether (x, y) is inside or outside of the basisConvexHull
	const int basisConvexHullSize = basisConvexHull.size();

	int crossCount = 0;
	for (int i = 0; i < basisConvexHullSize; i++) {
		SketchPoint u = basisConvexHull[i];
		SketchPoint v = basisConvexHull[(i + 1)%basisConvexHullSize];
		if (crossLines(p, u, v)) crossCount++;
	}

	if (crossCount%2 == 0) return false;
	else return true;
}

// Filter the interpolated points in depth
//	1. Construct PointCloud class on targetVertices
//	2. Cast a ray for each discretized grid and select candidate points
//	3. Detect depth with DBSCAN
std::vector<Vertex> ToolUtil::filterWithDepth(std::vector<Vertex> &targetVertices) {
	// TODO: Implement depth filtering 
	std::vector<Vertex> filteredVertices;
	for (Vertex v: targetVertices) {
		// Cast a ray
		Ray ray(camera, glm::vec3(v.x, v.y, v.z));
		Ray::Hit hit = ray.castPointToScreen();
		glm::vec2 pos_screen = camera->mapWorldToScreen(hit.pos);
		if (insideSketch(SketchPoint{pos_screen.x, pos_screen.y})) {
			filteredVertices.emplace_back(v);
		}
	}
	return filteredVertices;
}

// Filter the interpolated points in voxel, whose side is equal to averageDistance
//	1. Compute whether each point is a candidate point
//	2. Search the point cloud and target points for nearest neighbors for each points
//	3. Select candidate points for each voxel.
std::vector<Vertex> ToolUtil::filterWithVoxel(std::vector<Vertex> &targetVertices) {
	const double voxelSide = pointCloud->getAverageDistance();

  // map for voxel filter
  //  - std::tupple<double, double, double>:  the index of the voxel
  //  - std::pair<double, int>: the distance between the center point and the current 
  //                            candidate point and the index of the current candidate point
  std::map<std::tuple<int, int, int>, std::pair<double, int>> voxels;

	std::vector<bool> isCandidatePoint(targetVertices.size());
	for (size_t i = 0; i < targetVertices.size(); i++) {
		// Calculate voxel index
		Vertex v = targetVertices[i];
    std::tuple<int, int, int> idx = {
      std::floor(v.x/voxelSide),
      std::floor(v.y/voxelSide),
      std::floor(v.z/voxelSide)
    };

		// Set current candidate information
		std::pair<double, int> currentCandidate = {1e5, i};
		if (voxels.count(idx) != 0) {
			currentCandidate = voxels[idx];
		}
    double currentCandidateDist = currentCandidate.first;
    int currentCandidateIdx = currentCandidate.second;

    // Update the voxels information
    glm::vec3 voxelBasis = glm::vec3(
      (float)std::get<0>(idx)*voxelSide,
      (float)std::get<1>(idx)*voxelSide,
      (float)std::get<2>(idx)*voxelSide
    );
    glm::vec3 voxelCenter = voxelBasis + 0.5f*glm::vec3(voxelSide, voxelSide, voxelSide);
    double currentDist = glm::length(glm::vec3(v.x, v.y, v.z) - voxelCenter);
    if (currentDist < currentCandidateDist) {
      voxels[idx] = { currentDist, i };
      isCandidatePoint[currentCandidateIdx] = false;
      isCandidatePoint[i] = true;
    }
	}

	// Search for nearest neighbors for each targetVertices
  // If candidate points are in the same voxel with points 
  // of the point cloud, then skip it.
	std::vector<Vertex> filteredVertices;
	std::vector<Vertex>* verticesPtr = pointCloud->getVertices();
  for (size_t i = 0; i < targetVertices.size(); i++) {
    if (!isCandidatePoint[i]) continue;

		Vertex v = targetVertices[i];
    std::tuple<int, int, int> v_idx = {
      std::floor(v.x/voxelSide),
      std::floor(v.y/voxelSide),
      std::floor(v.z/voxelSide)
    };

    // Search the point cloud for nearest neighbors of point p.
    // This search is based on radiusSearch.
    std::vector<int>    hitPointIndices;
    std::vector<float>  hitPointDistances;
    int hitPointCount = pointCloud->getOctree()->radiusSearch(
      pcl::PointXYZ(v.x, v.y, v.z),
      voxelSide,
      hitPointIndices,
      hitPointDistances
    );

    // Check whether some points are already in the same voxel.
    bool pointsInSameVoxel = false;
    for (int i = 0; i < hitPointCount; i++) {
      Vertex hitPoint = (*verticesPtr)[hitPointIndices[i]];
      std::tuple<int, int, int> hit_idx = {
        std::floor(hitPoint.x/voxelSide),
        std::floor(hitPoint.y/voxelSide),
        std::floor(hitPoint.z/voxelSide)
      };
      if (hit_idx == v_idx) pointsInSameVoxel = true;
    }

    // If no point is already in the same voxel, then add it.
    if (!pointsInSameVoxel) filteredVertices.emplace_back(v);
  }

	return filteredVertices;
}

// Calculate averageDistance casted onto the screen
double ToolUtil::calcCastedAverageDist() {
	double screenDist = camera->getNearClip();
	double objectDist = glm::length(camera->getWorldPosition());
	return (pointCloud->getAverageDistance()/objectDist)*screenDist;
}

/* Get Member Variables */
PointCloud* ToolUtil::getPointCloud() {	return pointCloud; }
Sketch* ToolUtil::getSketch() { return sketch; }
Camera* ToolUtil::getCamera() { return camera; }

std::set<int>* ToolUtil::getSurfacePointsIndex() { return &surfacePointsIndex; }
std::vector<Vertex>* ToolUtil::getBasisPoints() { return &basisPoints; }
std::vector<SketchPoint>* ToolUtil::getSketchPoints() { return &sketchPoints; }
std::vector<SketchPoint>* ToolUtil::getBasisConvexHull() { return &basisConvexHull; }

PointCloud* ToolUtil::getBasisPointCloud() { return &basisPointCloud; }

// Calculate CCW value
//  - ccw > 0.0: left turn
//  - ccw < 0.0: right turn
//  - ccw = 0.0: parallel
double ToolUtil::calcCCW(SketchPoint p, SketchPoint a, SketchPoint b) {
	glm::dvec2 p_vec(p.x, p.y);
	glm::dvec2 a_vec(a.x, a.y);
	glm::dvec2 b_vec(b.x, b.y);

	glm::dvec2 u = a_vec - p_vec;
	glm::dvec2 v = b_vec - p_vec;
	return u.x*v.y - v.x*u.y;
}

// Calculate the convex hull of basisPoints O(n\log{n})
//  1. Find point P with the lowest y-coordinate.
//  2. Sort the points in increasing order of the angle
//     they and the point P make with the x-axis.
//     (Break ties by increasing distance.)
//  3. Traversing all points considering "left turn" and "right turn"
//
// implemented referencing https://en.wikipedia.org/wiki/Graham_scan
void ToolUtil::computeBasisConvexHull() {

}

// Return whether half-line from (x, y) coresses u-v
bool ToolUtil::crossLines(SketchPoint p, SketchPoint u, SketchPoint v) {
  const double EPS = 1e-5; 

  // If u, v are too close to p, then offset. 
  if (std::abs(u.y - p.y) < EPS) {
    if (u.y - p.y >= 0.0)  u.y += EPS;
    else u.y -= EPS;
  }
  if (std::abs(v.y - p.y) < EPS) {
    if (v.y - p.y >= 0.0)  v.y += EPS;
    else v.y -= EPS;
  }

  // If (u, v) is parallel to x-axis, then skip it.
  if (std::abs(u.y - p.y) < EPS && std::abs(v.y - p.y) < EPS) return false;

  // If u, v are in the same side of the half-line, then skip it.
  if ((u.y - p.y)*(v.y - p.y) >= 0.0) return false;

  // If (u, v) doesn't intersect the half-line, then skip it.
  double crossX = u.x + (v.x - u.x)*std::abs(p.y - u.y)/std::abs(v.y - u.y);
  if (crossX < p.x) return false;
  else return true;
}
