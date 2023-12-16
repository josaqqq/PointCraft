#include "polyscope/polyscope.h"

#include "polyscope/point_cloud.h"
#include "polyscope/view.h"

#include <map>

#include "constants.hpp"
#include "interpolation_tool.hpp"
#include "surface.hpp"
#include "cluster.hpp"

void InterpolationTool::launchToolOperation() {
  polyscope::view::moveScale = 0.0;

  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    draggingEvent();
  } else {
    releasedEvent();
  }
}

void InterpolationTool::draggingEvent() {
  ImGuiIO &io = ImGui::GetIO();
  ImVec2 mousePos = ImGui::GetMousePos();
  double xPos = io.DisplayFramebufferScale.x * mousePos.x;
  double yPos = io.DisplayFramebufferScale.y * mousePos.y;

  // Cast a ray to screen
  Ray ray(xPos, yPos);
  Ray::Hit hitInfo = ray.castPointToPlane(getScreen());
  assert(hitInfo.hit);
  addSketchPoint(hitInfo.pos);

  // Update surfacePointsIndex
  updateSurfacePoints(xPos, yPos, SurfacePointNum);
  removePointCloud(SurfacePointName);
  registerSurfacePointsAsPointCloud(SurfacePointName);

  // Register:
  //  - sketch
  registerSketchPointsAsCurveNetworkLine(SketchPrefix);
}

void InterpolationTool::releasedEvent() {
  if (getSketchPoints()->size() == 0) return;

  // Find basis points for the surface reconstruction.
  findBasisPoints();
  if (getBasisPointsIndex()->size() == 0) {
    // Remove:
    //  - surface points
    //  - sketch
    removePointCloud(SurfacePointName);
    removeCurveNetworkLine(SketchPrefix);
    // Reset all member variables.
    resetSketch();
    std::cout << "WARNING: No basis point was found." << std::endl;
    return;
  }
  
  // Fetch basis points information from the point cloud.
  std::set<int> *basisPointsIndexPtr    = getBasisPointsIndex();
  std::vector<glm::dvec3> *verticesPtr  = getPointCloud()->getVertices();
  std::vector<glm::dvec3> *normalsPtr   = getPointCloud()->getNormals();

  std::vector<glm::dvec3> basisPoints, basisNormals;
  for (int idx: *basisPointsIndexPtr) {
    basisPoints.push_back((*verticesPtr)[idx]);
    basisNormals.push_back((*normalsPtr)[idx]);
  }

  // Compute Poisson Surface Reconstruction
  std::vector<glm::dvec3>           poissonPoints;
  std::vector<std::vector<size_t>>  poissonFaces;
  Surface poissonSurface("Interpolation: PSR", &basisPoints, &basisNormals);
  std::tie(poissonPoints, poissonFaces) = poissonSurface.reconstructPoissonSurface(getPointCloud()->getAverageDistance(), false);
  if (poissonPoints.size() == 0) {
    // Remove:
    //  - surface points
    //  - sketch
    removePointCloud(SurfacePointName);
    removeCurveNetworkLine(SketchPrefix);
    // Reset all member variables.
    resetSketch();
    std::cout << "WARNING: No mesh was reconstructed with Poisson Surface Reconstruction." << std::endl;
    return;
  }

  // Calculate the normals of the reconstructed surface.
  std::vector<glm::dvec3> poissonNormals = calculateSurfaceNormals(poissonPoints, poissonFaces);

  // Filter the reconstructed surface with depth
  std::set<int> depthFilteredIndex = filterWithDepth(poissonPoints, poissonNormals);
  std::vector<glm::dvec3> depthFilteredPoints, depthFilteredNormals;
  for (int idx: depthFilteredIndex) {
    depthFilteredPoints.push_back(poissonPoints[idx]);
    depthFilteredNormals.push_back(poissonNormals[idx]);
  }

  // Filter the reconstructed surface with voxel
  std::set<int> voxelFilteredIndex = filterWithVoxel(depthFilteredPoints);
  std::vector<glm::dvec3> voxelFilteredPoints, voxelFilteredNormals;
  for (int idx: voxelFilteredIndex) {
    voxelFilteredPoints.push_back(depthFilteredPoints[idx]);
    voxelFilteredNormals.push_back(depthFilteredNormals[idx]);
  }

  // Register:
  //  - basis points
  //  - interpolated points
  registerBasisPointsAsPointCloud("Basis Points");
  renderInterpolatedPoints(voxelFilteredPoints, voxelFilteredNormals);

  // Remove:
  //  - surface points
  //  - sketch
  removePointCloud(SurfacePointName);
  removeCurveNetworkLine(SketchPrefix);
  
  // 1. Add the interpolated points
  // 2. Update point cloud
  //    - update environments
  //    - update octree
  //    - render points and normals
  getPointCloud()->addPoints(voxelFilteredPoints, voxelFilteredNormals);
  getPointCloud()->updatePointCloud(true);

  // Reset all member variables.
  resetSketch();
}

// Register new vertices and normals as point cloud
void InterpolationTool::renderInterpolatedPoints(
  std::vector<glm::dvec3> &newV,
  std::vector<glm::dvec3> &newN
) {
  polyscope::PointCloud* interpolatedCloud = polyscope::registerPointCloud("Interpolated Points", newV);
  interpolatedCloud->setPointColor(glm::dvec3(1.0, 0.0, 0.0));
  interpolatedCloud->setPointRadius(BasisPointRadius);
  interpolatedCloud->setEnabled(false);

  polyscope::PointCloudVectorQuantity* interpolatedVectorQuantity = interpolatedCloud->addVectorQuantity(NormalName, newN);
  interpolatedVectorQuantity->setVectorColor(glm::dvec3(1.0, 0.0, 0.0));
  interpolatedVectorQuantity->setVectorLengthScale(NormalLength);
  interpolatedVectorQuantity->setVectorRadius(NormalRadius * 1.1);
  interpolatedVectorQuantity->setEnabled(NormalEnabled);
  interpolatedVectorQuantity->setMaterial(NormalMaterial);
}

// Calculate normals of the surface points
std::vector<glm::dvec3> InterpolationTool::calculateSurfaceNormals(
  std::vector<glm::dvec3> &surfacePoints,
  std::vector<std::vector<size_t>> &surfaceFaces
) {
  const int surfacePointsSize = surfacePoints.size();
  const int surfaceFacesSize = surfaceFaces.size();

  // Calculate the points' average normals
  std::map<int, glm::dvec3> indexToNormalSum;
  std::map<int, int>        indexToAdjacentCount;
  // Normals of surfacePoints
  for (int i = 0; i < surfaceFacesSize; i++) {
    // Calculate the normal of the triangle
    glm::dvec3 u = surfacePoints[surfaceFaces[i][0]];
    glm::dvec3 v = surfacePoints[surfaceFaces[i][1]];
    glm::dvec3 w = surfacePoints[surfaceFaces[i][2]];

    glm::dvec3 n = glm::normalize(glm::cross(v - u, w - u));
    for (int j = 0; j < 3; j++) {
      int vertexIdx = surfaceFaces[i][j];
      indexToNormalSum[vertexIdx] += n;
      indexToAdjacentCount[vertexIdx]++;
    }
  }

  std::vector<glm::dvec3> surfaceNormals(surfacePointsSize);
  for (int i = 0; i < surfacePointsSize; i++) {
    surfaceNormals[i] = indexToNormalSum[i]/(double)indexToAdjacentCount[i];
  }

  return surfaceNormals;
}

// Filter the interpolated points with depth
//  1. Cast interpolated points onto the screen plane.
//  2. Construct octree for the casted interpolated points.
//  3. Search for a candidate point for each discretized grid.
//    - Only points that their normals are directed to cameraOrig.
//  4. Detect depth with DBSCAN
//
//  - surfacePoints:  Positions of the interpolated surface points
//  - surfaceNormals: Normals of the interpolated surface points
std::set<int> InterpolationTool::filterWithDepth(
  std::vector<glm::dvec3> &surfacePoints,
  std::vector<glm::dvec3> &surfaceNormals
) {
  const int surfacePointsSize = surfacePoints.size();

  // Cast reconstructed surface onto the screen plane.
  std::vector<glm::dvec3> pointsInWorldCoord;
  std::vector<glm::dvec3> normalsInWorldCoord;
  std::vector<glm::dvec3> pointsCastedOntoScreen;
  // Cast surface points onto screen
  for (int i = 0; i < surfacePointsSize; i++) {
    glm::dvec3 p = surfacePoints[i];
    glm::dvec3 pn = surfaceNormals[i];
    
    // Cast a ray from p to cameraOrig onto screen.
    Ray ray(p, getCameraOrig());
    Ray::Hit hitInfo = ray.castPointToPlane(getScreen());
    assert(hitInfo.hit);

    pointsInWorldCoord.push_back(p);
    normalsInWorldCoord.push_back(pn);
    pointsCastedOntoScreen.push_back(getScreen()->mapCoordinates(hitInfo.pos));
  }

  // Construct octree for the casted surface points
  pcl::PointCloud<pcl::PointXYZ>::Ptr castedSurfaceCloud(new pcl::PointCloud<pcl::PointXYZ>);
  castedSurfaceCloud->points.resize(surfacePointsSize);
  for (int i = 0; i < surfacePointsSize; i++) {
    castedSurfaceCloud->points[i].x = pointsCastedOntoScreen[i].x;
    castedSurfaceCloud->points[i].y = pointsCastedOntoScreen[i].y;
    castedSurfaceCloud->points[i].z = pointsCastedOntoScreen[i].z;
  }
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(OctreeResolution);
  octree.setInputCloud(castedSurfaceCloud);
  octree.addPointsFromInputCloud();

  // Search for a candidate point for each discretized grid.
  const double INF = 1e5;
  double min_x = INF, max_x = -INF;
  double min_y = INF, max_y = -INF;
  for (int i = 0; i < surfacePointsSize; i++) {
    glm::dvec3 mapped_p = pointsCastedOntoScreen[i];

    min_x = std::min(min_x, mapped_p.x);
    max_x = std::max(max_x, mapped_p.x);
    min_y = std::min(min_y, mapped_p.y);
    max_y = std::max(max_y, mapped_p.y);
  }

  const double castedAverageDist = calcCastedAverageDist();
  std::set<int>           candidatePointsIndexSet;
  std::vector<glm::dvec3> hasCloserNeighborPoints;
  for (double x = min_x; x < max_x; x += castedAverageDist) {
    for (double y = min_y; y < max_y; y += castedAverageDist) {
      if (!insideSketch(x, y)) continue;
      if (!insideBasisConvexHull(x, y)) continue;

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
        glm::dvec3 p = pointsInWorldCoord[hitPointIdx];
        glm::dvec3 pn = normalsInWorldCoord[hitPointIdx];

        double curDepth = glm::length(p - getCameraOrig());
        if (curDepth < minDepth) {
          minDepth = curDepth;
          minDepthIdx = hitPointIdx;
        }
      }

      // Update the buffer vectors
      for (int i = 0; i < hitPointCount; i++) {
        int hitPointIdx = hitPointIndices[i];
        glm::dvec3 p = pointsInWorldCoord[hitPointIdx];
        glm::dvec3 pn = normalsInWorldCoord[hitPointIdx];

        // Discard points that their normals are not directed to cameraOrig.
        if (glm::dot(pn, p - getCameraOrig()) >= 0.0) continue;

        if (hitPointIdx == minDepthIdx) {
          candidatePointsIndexSet.insert(hitPointIdx);
        } else {
          hasCloserNeighborPoints.push_back(p);
        }
      }
    }
  }

  // Register points that has closer nearest neighbors as point cloud.
  polyscope::PointCloud* hasCloserNeighborCloud = polyscope::registerPointCloud("HasCloserNeighbor(surface)", hasCloserNeighborPoints);
  hasCloserNeighborCloud->setPointRadius(BasisPointRadius);
  hasCloserNeighborCloud->setEnabled(false);

  // Detect depth with DBSCAN
  Clustering clustering(&candidatePointsIndexSet, &pointsInWorldCoord, "surface");
  std::set<int> clusteredPointsIndex = clustering.executeClustering(
    DBSCAN_SearchRange*getPointCloud()->getAverageDistance(),
    DBSCAN_MinPoints,
    CLUSTER_MIN_DEPTH
  );

  return clusteredPointsIndex;
}