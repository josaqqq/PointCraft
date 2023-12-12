#include "polyscope/polyscope.h"

#include "polyscope/view.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/curve_network.h"

#include <pcl/point_cloud.h>

#include <glm/gtx/hash.hpp>

#include <stack>

#include "sketch_tool.hpp"
#include "constants.hpp"
#include "cluster.hpp"

/*
  Manage functions
*/
// Initialize screen information
void SketchTool::initSketch() {
  screenDist = polyscope::view::nearClipRatio * ScreenOffset;
  cameraOrig = polyscope::view::getCameraWorldPosition();
  cameraDir = polyscope::view::screenCoordsToWorldRay(
    glm::vec2(polyscope::view::windowWidth/2, polyscope::view::windowHeight/2)
  );
  glm::dvec3 screenOrig = cameraOrig + screenDist*cameraDir;

  // Plane on screenDist
  screen = Plane(screenOrig, cameraDir);
}

// Reset all member variables.
void SketchTool::resetSketch() {
  *currentMode = 0;

  screenDist = 0.0;
  cameraOrig = glm::dvec3(0.0, 0.0, 0.0);
  cameraDir = glm::dvec3(0.0, 0.0, 0.0);
  screen = Plane();

  // Reset member variables (vector and set)
  resetSurfacePointsIndex();
  resetSketchPoints();
  resetBasisPointsIndex();
  resetMappedBasisConvexHull();
}

/*
  Viewer functions
*/
void SketchTool::registerSurfacePointsAsPointCloud(std::string name) {
  // Show surfacePoints
  std::vector<glm::dvec3> points;

  std::vector<glm::dvec3> *verticesPtr = pointCloud->getVertices();
  for (int idx: surfacePointsIndex) {
    points.push_back((*verticesPtr)[idx]);
  }

  polyscope::PointCloud* patchCloud = polyscope::registerPointCloud(name, points);
  patchCloud->setPointColor(SurfacePointColor);
  patchCloud->setPointRadius(SurfacePointRadius);
  patchCloud->setMaterial(SurfaceMaterial);
  patchCloud->setEnabled(true);
}
void SketchTool::registerSketchPointsAsPointCloud(std::string name) {
  // Show sketchPoints
  polyscope::PointCloud* patchCloud = polyscope::registerPointCloud(name, sketchPoints);
  patchCloud->setPointColor(BasisPointColor);
  patchCloud->setPointRadius(BasisPointRadius);
  patchCloud->setEnabled(false);
}
void SketchTool::registerBasisPointsAsPointCloud(std::string name) {
  // Show basisPoints
  std::vector<glm::dvec3> points;
  std::vector<glm::dvec3> normals;

  std::vector<glm::dvec3> *verticesPtr = pointCloud->getVertices();
  std::vector<glm::dvec3> *normalsPtr = pointCloud->getNormals();
  for (int idx: basisPointsIndex) {
    points.push_back((*verticesPtr)[idx]);
    normals.push_back((*normalsPtr)[idx]);
  }

  polyscope::PointCloud* patchCloud = polyscope::registerPointCloud(name, points);
  patchCloud->setPointColor(BasisPointColor);
  patchCloud->setPointRadius(pointCloud->getAverageDistance()/2.0);
  patchCloud->setEnabled(false);

  polyscope::PointCloudVectorQuantity* patchVectorQuantity = patchCloud->addVectorQuantity(NormalName, normals);
  patchVectorQuantity->setVectorColor(BasisPointColor);
  patchVectorQuantity->setVectorLengthScale(pointCloud->getAverageDistance());
  patchVectorQuantity->setVectorRadius(pointCloud->getAverageDistance()/10.0);
  patchVectorQuantity->setEnabled(NormalEnabled);
  patchVectorQuantity->setMaterial(NormalMaterial);
}
void SketchTool::removePointCloud(std::string name) {
  polyscope::removePointCloud(name, false);
}

// Register/Remove curve network line with name.
// Be aware that the curve network with 
// the same name is overwritten
void SketchTool::registerSketchPointsAsCurveNetworkLine(std::string name) {
  std::vector<glm::dvec3> sketch = sketchPoints;
  if (sketchPoints.size() == 1) {
    sketch.push_back(sketchPoints[0]);
  };

  polyscope::CurveNetwork* curveNetwork = polyscope::registerCurveNetworkLine(name, sketch);
  curveNetwork->setColor(CurveNetworkColor);
  curveNetwork->setRadius(CurveNetworkRadius);  
  curveNetwork->setMaterial(CurveNetworkMaterial);
}
void SketchTool::removeCurveNetworkLine(std::string name) {
  polyscope::removeCurveNetwork(name, false);
}

// Register/Remove curve network loop with name.
// Be aware that the curve network with 
// the same name is overwritten
void SketchTool::registerSketchPointsAsCurveNetworkLoop(std::string name) {
  polyscope::CurveNetwork* curveNetwork = polyscope::registerCurveNetworkLoop(name, sketchPoints);
  curveNetwork->setColor(CurveNetworkColor);
  curveNetwork->setRadius(CurveNetworkRadius);
}
void SketchTool::removeCurveNetworkLoop(std::string name) {
  polyscope::removeCurveNetwork(name, false);
}

// Display voxels for each specified point.
void SketchTool::displayVoxels(std::vector<glm::dvec3> &points) {
  // Voxel size is averageDistance per side
  const double voxelSide = pointCloud->getAverageDistance();

  // Collect the indices in the voxel grid of the points
  std::set<std::tuple<int, int, int>> voxelIndices;
  for (glm::dvec3 p: points) {
    std::tuple<int, int, int> idx = {
      std::floor(p.x/voxelSide),
      std::floor(p.y/voxelSide),
      std::floor(p.z/voxelSide)
    };
    voxelIndices.insert(idx);
  }

  // Compute the registered mesh
  std::vector<std::vector<bool>> positionOffset = {
    { false, false, false },

    { true, false, false },
    { false, true, false },
    { false, false, true },

    { true, true, false },
    { false, true, true },
    { true, false, true },

    { true, true, true },
  };
  std::vector<std::vector<size_t>> faceIndices = {
    { 1, 0, 2 },
    { 1, 2, 4 },

    { 0, 1, 3 },
    { 3, 1, 6 },
    
    { 1, 4, 6 },
    { 4, 7, 6 },
    
    { 7, 4, 2 },
    { 7, 2, 5 },

    { 3, 6, 5 },
    { 5, 6, 7 },
    
    { 0, 3, 2 },
    { 2, 3, 5 },
  };

  std::vector<glm::dvec3> meshV;
  std::vector<std::vector<size_t>> meshF;
  for (std::tuple<int, int, int> idx: voxelIndices) {
    // Compute meshF
    size_t offset = meshV.size();
    for (size_t i = 0; i < faceIndices.size(); i++) {
      std::vector<size_t> faceIdx;
      for (int j = 0; j < 3; j++) faceIdx.push_back(offset + faceIndices[i][j]);
      meshF.push_back(faceIdx);
    }
    
    // Compute meshV
    for (size_t i = 0; i < positionOffset.size(); i++) {
      glm::dvec3 p = glm::dvec3(
        voxelSide*static_cast<double>(std::get<0>(idx)),
        voxelSide*static_cast<double>(std::get<1>(idx)),
        voxelSide*static_cast<double>(std::get<2>(idx))
      );
      for (int j = 0; j < 3; j++) {
        if (positionOffset[i][j]) p[j] += voxelSide;
      }
      meshV.push_back(p);
    }
  }

  // Register meshV, meshF as SurfaceMesh
  polyscope::SurfaceMesh *voxelMesh = polyscope::registerSurfaceMesh("Voxel Mesh", meshV, meshF);
  voxelMesh->setSurfaceColor({ 0.000, 153.0/255.0, 119.0/255.0 });
  voxelMesh->setMaterial("clay");
}

/*
  Geometry functions
*/
// Compute the surface points where mouse is currently hovering
//  - xPos: io.DisplayFramebufferScale.x * mousePos.x
//  - xPos: io.DisplayFramebufferScale.y * mousePos.y
//  - K_size: the selected nearest neighbors size
void SketchTool::updateSurfacePoints(double xPos, double yPos, int K_size) {
  // Cast a ray to pointCloud
  Ray ray(xPos, yPos);
    // Search radius is doubled because the user 
    // should reference the boundary of the pseudo surface.
  Ray::Hit hitInfo = ray.searchNearestNeighbor(
    pointCloud,
    pointCloud->getAverageDistance() * 2.0  
  );
  if (!hitInfo.hit) return;

  // Search the point cloud for K-nearest-neighbors of "center point"
  glm::dvec3 rayDir = hitInfo.rayDir;
  glm::dvec3 center = cameraOrig + glm::dot(hitInfo.pos - cameraOrig, rayDir)*rayDir;

  // Search pointCloud for nearest neighbors
  std::vector<int>    hitPointIndices;
  std::vector<float>  hitPointDistances;
  int hitPointCount = pointCloud->getOctree()->nearestKSearch(
    pcl::PointXYZ(center.x, center.y, center.z),
    K_size,
    hitPointIndices,
    hitPointDistances
  );

  std::vector<glm::dvec3> *normalsPtr = pointCloud->getNormals();
  for (int i = 0; i < hitPointCount; i++) {
    int idx = hitPointIndices[i];

    // If the normal does not face rayDir, then skip it.
    glm::dvec3 pn = (*normalsPtr)[idx];
    if (glm::dot(pn, rayDir) >= 0.0) continue;

    // Add nearest neighbor to surfacePointsIndex
    surfacePointsIndex.insert(idx);
  }
}

// Cast the specified point to screen.
void SketchTool::addSketchPoint(glm::dvec3 p) {
  sketchPoints.push_back(p);
}

// Find basis points.
//  - Cast points of the point cloud onto the screen plane.
//  - Construct octree for the casted surface points
//  - Search for a candidate point for each discretized grid.
//    - Only points that their normals are directed to cameraOrig
//  - Detect depth with DBSCAN
void SketchTool::findBasisPoints() {
  // Point cloud vertices and normals
  std::vector<glm::dvec3> *verticesPtr = pointCloud->getVertices();
  std::vector<glm::dvec3> *normalsPtr = pointCloud->getNormals();

  // Cast points of the point cloud onto the screen plane.
  std::vector<glm::dvec3> pointsMappedOntoScreen;
  for (size_t i = 0; i < (*verticesPtr).size(); i++) {
    glm::dvec3 p = (*verticesPtr)[i];

    // Cast a ray from p to cameraOrig onto screen.
    Ray ray(p, cameraOrig);
    Ray::Hit hitInfo = ray.castPointToPlane(&screen);
    assert(hitInfo.hit == true);
    pointsMappedOntoScreen.push_back(screen.mapCoordinates(hitInfo.pos));
  }
  const int castedPointSize = pointsMappedOntoScreen.size();

  // Construct octree for the casted points
  pcl::PointCloud<pcl::PointXYZ>::Ptr castedCloud(new pcl::PointCloud<pcl::PointXYZ>);
  castedCloud->points.resize(castedPointSize);
  for (int i = 0; i < castedPointSize; i++) {
    castedCloud->points[i].x = pointsMappedOntoScreen[i].x;
    castedCloud->points[i].y = pointsMappedOntoScreen[i].y;
    castedCloud->points[i].z = pointsMappedOntoScreen[i].z;
  }
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(OctreeResolution);
  octree.setInputCloud(castedCloud);
  octree.addPointsFromInputCloud();

  // Search for a candidate point for each discretized grid.
  const double INF = 1e5;
  double min_x = INF, max_x = -INF;
  double min_y = INF, max_y = -INF;
  for (size_t i = 0; i < sketchPoints.size(); i++) {
    glm::dvec3 mappedSketchPoint = screen.mapCoordinates(sketchPoints[i]);

    min_x = std::min(min_x, mappedSketchPoint.x);
    max_x = std::max(max_x, mappedSketchPoint.x);
    min_y = std::min(min_y, mappedSketchPoint.y);
    max_y = std::max(max_y, mappedSketchPoint.y);
  }

  const double      castedAverageDist = calcCastedAverageDist();
  std::set<int>     candidatePointsIndexSet;
  std::vector<glm::dvec3> hasCloserNeighborPoints;
  for (int idx: surfacePointsIndex) {
    candidatePointsIndexSet.insert(idx);
  }
  for (double x = min_x; x < max_x; x += castedAverageDist) {
    for (double y = min_y; y < max_y; y += castedAverageDist) {
      if (!insideSketch(x, y)) continue;

      // Search for nearest neighbors with octree
      std::vector<int>    hitPointIndices;
      std::vector<float>  hitPointDistances;
      int hitPointCount = octree.radiusSearch(
        pcl::PointXYZ(x, y, 0.0),
        castedAverageDist,
        hitPointIndices,
        hitPointDistances
      );

      // Get the index of the min depth point
      double minDepth = 1e5;
      int minDepthIdx = -1;
      for (int i = 0; i < hitPointCount; i++) {
        int hitPointIdx = hitPointIndices[i];
        glm::dvec3 p = (*verticesPtr)[hitPointIdx];
        glm::dvec3 pn = (*normalsPtr)[hitPointIdx];

        double curDepth = glm::length(p - cameraOrig);
        if (curDepth < minDepth) {
          minDepth = curDepth;
          minDepthIdx = hitPointIdx;
        }
      }

      // Update the buffer vectors
      for (int i = 0; i < hitPointCount; i++) {
        int hitPointIdx = hitPointIndices[i];
        glm::dvec3 p = (*verticesPtr)[hitPointIdx];
        glm::dvec3 pn = (*normalsPtr)[hitPointIdx];

        // Discard points that their normals are not directed to cameraOrig.
        if (glm::dot(pn, p - cameraOrig) >= 0.0) continue;

        if (hitPointIdx == minDepthIdx) {
          candidatePointsIndexSet.insert(hitPointIdx);
        } else {
          hasCloserNeighborPoints.push_back(p);
        }
      }
    }
  }

  // Register points that has closer nearest neighbors as point cloud.
  polyscope::PointCloud* hasCloserNeighborCloud = polyscope::registerPointCloud("HasCloserNeighbor(basis)", hasCloserNeighborPoints);
  hasCloserNeighborCloud->setPointRadius(BasisPointRadius);
  hasCloserNeighborCloud->setEnabled(false);

  // Depth detection with DBSCAN
  Clustering clustering(&candidatePointsIndexSet, verticesPtr, "basis");
  basisPointsIndex = clustering.executeClustering(
    DBSCAN_SearchRange*pointCloud->getAverageDistance(),
    DBSCAN_MinPoints,
    CLUSTER_MAX_SIZE
  );
}

// Find all basis points inside the sketch
//  - Cast points of the point cloud onto the screen plane.
//  - Judge inside/outside of the sketch.
//  - Detect depth with DBSCAN.
void SketchTool::findAllBasisPoints() {
  // Point cloud vertices
  std::vector<glm::dvec3> *verticesPtr = pointCloud->getVertices();

  // Cast points of the point cloud onto the screen plane.
  std::vector<glm::dvec3> pointsMappedOntoScreen;
  for (size_t i = 0; i < (*verticesPtr).size(); i++) {
    glm::dvec3 p = (*verticesPtr)[i];

    // Cast a ray from p to cameraOrig onto screen.
    Ray ray(p, cameraOrig);
    Ray::Hit hitInfo = ray.castPointToPlane(&screen);
    assert(hitInfo.hit == true);
    pointsMappedOntoScreen.push_back(screen.mapCoordinates(hitInfo.pos));
  }
  const int castedPointSize = pointsMappedOntoScreen.size();

  // Judge inside/outside of the sketch
  std::set<int> candidatePointsIndexSet;
  for (int i = 0; i < castedPointSize; i++) {
    glm::dvec3 mapped_p = pointsMappedOntoScreen[i];
    if (!insideSketch(mapped_p.x, mapped_p.y)) continue;

    candidatePointsIndexSet.insert(i);
  }

  // Depth detection with DBSCAN
  Clustering clustering(&candidatePointsIndexSet, verticesPtr, "basis");
  basisPointsIndex = clustering.executeClustering(
    DBSCAN_SearchRange*pointCloud->getAverageDistance(),
    DBSCAN_MinPoints,
    CLUSTER_MIN_DEPTH
  );
}

// Check whether (x, y) is inside or outside of the sketch.
//  1.  Draw a half-line parallel to the x-axis from a point.
//  2.  Determine that if there are an odd number of intersections
//      between this half-line and the polygon, it is inside, and if 
//      there are an even number, it is outside.
bool SketchTool::insideSketch(double x, double y) {
  const int sketchPointsSize = sketchPoints.size();

  int crossCount = 0;
  for (int i = 0; i < sketchPointsSize; i++) {
    glm::dvec3 mappedU = screen.mapCoordinates(sketchPoints[i]);
    glm::dvec3 mappedV = screen.mapCoordinates(sketchPoints[(i + 1) % sketchPointsSize]);

    glm::dvec2 u = glm::dvec2(mappedU.x, mappedU.y);
    glm::dvec2 v = glm::dvec2(mappedV.x, mappedV.y);

    if (crossLines(x, y, u, v)) crossCount++;
  }

  if (crossCount%2 == 0) return false;
  else return true;
}

// Check whether (x, y) is inside or outside of the mappedBasisConvexHull.
//  1.  If haven't already calculated the convex hull of the basis points,
//      then calculate it.
//  2.  Draw a half-line parallel to the x-axis from a point.
//  3.  Determine that if there are an odd number of intersections
//      between this half-line and the polygon, it is inside, and if 
//      there are an even number, it is outside.
bool SketchTool::insideBasisConvexHull(double x, double y) {
  // If the size of basisPoints is less than 3, then return false.
  if (basisPointsIndex.size() < 3) return false;

  // If haven't already calculated the convex hull of the basis points, then calculate it.
  if (mappedBasisConvexHull.size() == 0) {
    calcBasisConvexHull();
  }

  // Check whether (x, y) is inside or outside of the mappedBasisConvexHull.
  const int basisConvexHullSize = mappedBasisConvexHull.size();

  int crossCount = 0;
  for (int i = 0; i < basisConvexHullSize; i++) {
    glm::dvec2 u = mappedBasisConvexHull[i];
    glm::dvec2 v = mappedBasisConvexHull[(i + 1) % basisConvexHullSize];

    if (crossLines(x, y, u, v)) crossCount++;
  }

  if (crossCount%2 == 0) return false;
  else return true;
}


// Filter the points on the interpolated surface.
// Considering points of the surface and the point cloud,
// select the candidate point of each voxel.
// Voxel size is averageDistance per side
//  - filteredPoints: Filtering target
std::set<int> SketchTool::filterWithVoxel(std::vector<glm::dvec3> &filteredPoints) {
  // Voxel size is averageDistance per side
  const double voxelSide = pointCloud->getAverageDistance();

  // map for voxel filter
  //  - std::tupple<double, double, double>:  the index of the voxel
  //  - std::pair<double, int>: the distance between the center point and the current 
  //                            candidate point and the index of the current candidate point
  std::map<std::tuple<int, int, int>, std::pair<double, int>> voxels;

  // Compute whether each surface point is a candidate point.
  std::vector<bool> isCandidatePoint(filteredPoints.size());
  for (size_t i = 0; i < filteredPoints.size(); i++) {
    glm::dvec3 p = filteredPoints[i];
    std::tuple<int, int, int> idx = {
      std::floor(p.x/voxelSide),
      std::floor(p.y/voxelSide),
      std::floor(p.z/voxelSide)
    };

    std::pair<double, int> currentCandidate = { 1e5, i };
    if (voxels.count(idx) != 0) {
      currentCandidate = voxels[idx];
    }

    double currentCandidateDist = currentCandidate.first;
    int currentCandidateIdx = currentCandidate.second;

    // Update the voxels information
    glm::dvec3 voxelBasis = glm::dvec3(
      (double)std::get<0>(idx)*voxelSide,
      (double)std::get<1>(idx)*voxelSide,
      (double)std::get<2>(idx)*voxelSide
    );
    glm::dvec3 voxelCenter = voxelBasis + 0.5d*glm::dvec3(voxelSide, voxelSide, voxelSide);
    double currentDist = glm::length(p - voxelCenter);
    if (currentDist < currentCandidateDist) {
      voxels[idx] = { currentDist, i };
      isCandidatePoint[currentCandidateIdx] = false;
      isCandidatePoint[i] = true;
    }
  }

  // Search for nearest neighbors for each filteredPoints
  // If candidate points are in the same voxel with points 
  // of the point cloud, then skip it.
  std::set<int> filteredIndex;
  std::vector<glm::dvec3> *verticesPtr = pointCloud->getVertices();
  for (size_t i = 0; i < filteredPoints.size(); i++) {
    if (!isCandidatePoint[i]) continue;

    glm::dvec3 p = filteredPoints[i];
    std::tuple<int, int, int> p_idx = {
      std::floor(p.x/voxelSide),
      std::floor(p.y/voxelSide),
      std::floor(p.z/voxelSide)
    };

    // Search the point cloud for nearest neighbors of point p.
    // This search is based on radiusSearch.
    std::vector<int>    hitPointIndices;
    std::vector<float>  hitPointDistances;
    int hitPointCount = pointCloud->getOctree()->radiusSearch(
      pcl::PointXYZ(p.x, p.y, p.z),
      voxelSide,
      hitPointIndices,
      hitPointDistances
    );

    // Check whether some points are already in the same voxel.
    bool pointsInSameVoxel = false;
    for (int i = 0; i < hitPointCount; i++) {
      glm::dvec3 hitPoint = (*verticesPtr)[hitPointIndices[i]];
      std::tuple<int, int, int> hit_idx = {
        std::floor(hitPoint.x/voxelSide),
        std::floor(hitPoint.y/voxelSide),
        std::floor(hitPoint.z/voxelSide)
      };
      if (hit_idx == p_idx) pointsInSameVoxel = true;
    }

    // If no point is already in the same voxel, then add it.
    if (!pointsInSameVoxel) filteredIndex.insert(i);
  }

  return filteredIndex;
}

// Calculate averageDistance casted onto the screen
double SketchTool::calcCastedAverageDist() {
  double objectDist = glm::length(cameraOrig);
  double castedAverageDist = pointCloud->getAverageDistance()*screenDist/objectDist;

  return castedAverageDist;
}

// Reset member variables (vector and set)
void SketchTool::resetSurfacePointsIndex() {
  surfacePointsIndex.clear();
}
void SketchTool::resetSketchPoints() {
  sketchPoints.clear();
}
void SketchTool::resetBasisPointsIndex() {
  basisPointsIndex.clear();
}
void SketchTool::resetMappedBasisConvexHull() {
  mappedBasisConvexHull.clear();
}

// Return the pointer to member variables.
PointCloud* SketchTool::getPointCloud() {
  return pointCloud;
}
glm::dvec3 SketchTool::getCameraOrig() {
  return cameraOrig;
}
glm::dvec3 SketchTool::getCameraDir() {
  return cameraDir;
}
Plane* SketchTool::getScreen() {
  return &screen;
}
std::set<int>* SketchTool::getSurfacePointsIndex() {
  return &surfacePointsIndex;
}
std::vector<glm::dvec3>* SketchTool::getSketchPoints() {
  return &sketchPoints;
}
std::set<int>* SketchTool::getBasisPointsIndex() {
  return &basisPointsIndex;
}
std::vector<glm::dvec2>* SketchTool::getMappedBasisConvexHull() {
  return &mappedBasisConvexHull;
}

// Extend sketched area by averageDistance casted onto the screen.
void SketchTool::extendSketchedArea() {
  // Calculate gravity point
  glm::dvec3 gravityPoint = glm::dvec3(0.0, 0.0, 0.0);
  for (glm::dvec3 p: sketchPoints) {
    gravityPoint += p;
  }
  gravityPoint /= static_cast<double>(sketchPoints.size());

  // Extend sketched area
  double castedAverageDist = calcCastedAverageDist();
  for (size_t i = 0; i < sketchPoints.size(); i++) {
    glm::dvec3 p = sketchPoints[i];

    // ATTENTION: Double the average distance, because the user 
    //            should sketch with reference to the boundary of pseudo surface.
    sketchPoints[i] = p + 2.0*castedAverageDist*glm::normalize(p - gravityPoint);
  }
}

// Calculate CCW value
//  - ccw > 0.0: left turn
//  - ccw < 0.0: right turn
//  - ccw == 0.0: parallel
double SketchTool::calcCCW(glm::dvec2 p, glm::dvec2 a, glm::dvec2 b) {
  glm::dvec2 u = a - p;
  glm::dvec2 v = b - p;
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
void SketchTool::calcBasisConvexHull() {
  const int basisPointsSize = basisPointsIndex.size();

  // Point cloud vertices
  std::vector<glm::dvec3> *verticesPtr = pointCloud->getVertices();

  // Initialize vector of mapped basis points.
  std::vector<glm::dvec2> mappedBasisPoints;
  for (int idx: basisPointsIndex) {
    glm::dvec3 p = (*verticesPtr)[idx];

    // Cast a ray from p to cameraOrig onto screen.
    Ray ray(p, cameraOrig);
    Ray::Hit hitInfo = ray.castPointToPlane(&screen);
    assert(hitInfo.hit == true);
    glm::dvec3 mappedP = screen.mapCoordinates(hitInfo.pos);
    mappedBasisPoints.push_back(glm::dvec2(mappedP.x, mappedP.y));
  }

  //  1. Find point P with the lowest y-coordinate.
  double min_y = mappedBasisPoints[0].y, minIndex = 0;
  for (int i = 1; i < basisPointsSize; i++) {
    glm::dvec2 p = mappedBasisPoints[i];
    if (p.y < min_y || (p.y == min_y && p.x < mappedBasisPoints[minIndex].x)) {
      min_y = p.y;
      minIndex = i;
    }
  }

  //  2. Sort the points in increasing order of the angle
  //     they and the point P make with the x-axis.
  //     (Break ties by increasing distance.)
  std::swap(mappedBasisPoints[0], mappedBasisPoints[minIndex]);
  glm::dvec2 P = mappedBasisPoints[0];
  std::sort(mappedBasisPoints.begin() + 1, mappedBasisPoints.end(), [&](const glm::dvec2 &l, const glm::dvec2 &r) {
    double ccw = calcCCW(P, l, r);
    if (ccw > 0.0) return true;
    else if (ccw < 0.0) return false;
    else {
      if (glm::length(l - P) < glm::length(r - P)) return true;
      else return false;
    }
  });

  //  3. Traversing all points considering "left turn" and "right turn"
  std::stack<glm::dvec2> pointStack;
  pointStack.push(mappedBasisPoints[0]);
  pointStack.push(mappedBasisPoints[1]);

  for (int i = 2; i < basisPointsSize; i++) {
    // p -> a -> b
    glm::dvec2 a = pointStack.top(); 
    pointStack.pop();

    glm::dvec2 b = mappedBasisPoints[i];

    while (!pointStack.empty() && calcCCW(pointStack.top(), a, b) <= 0.0) {
      a = pointStack.top();
      pointStack.pop();
    }
    pointStack.push(a);
    pointStack.push(b);
  }

  // Extract points from stack
  while (!pointStack.empty()) {
    mappedBasisConvexHull.push_back(pointStack.top());
    pointStack.pop();
  }
  std::reverse(mappedBasisConvexHull.begin(), mappedBasisConvexHull.end());
}

// Return whether half-line from (x, y) crosses u-v.
bool SketchTool::crossLines(double x, double y, glm::dvec2 u, glm::dvec2 v) {
  const double EPS = 1e-5; 

  // If u, v are too close to (x, y), then offset. 
  if (std::abs(u.y - y) < EPS) {
    if (u.y - y >= 0.0d)  u.y += EPS;
    else u.y -= EPS;
  }
  if (std::abs(v.y - y) < EPS) {
    if (v.y - y >= 0.0d)  v.y += EPS;
    else v.y -= EPS;
  }

  // If (u, v) is parallel to x-axis, then skip it.
  if (std::abs(u.y - y) < EPS && std::abs(v.y - y) < EPS) return false;

  // If u, v are in the same side of the half-line, then skip it.
  if ((u.y - y)*(v.y - y) >= 0.0d) return false;

  // If (u, v) doesn't intersect the half-line, then skip it.
  double crossX = u.x + (v.x - u.x)*std::abs(y - u.y)/std::abs(v.y - u.y);
  if (crossX < x) return false;
  else return true;
}