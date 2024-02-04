#include "polyscope/polyscope.h"

#include "polyscope/point_cloud.h"
#include "polyscope/view.h"

#include <map>
#include <fstream>

#include "constants.hpp"
#include "sketch_interpolation_tool.hpp"
#include "surface.hpp"
#include "cluster.hpp"

void SketchInterpolationTool::launchToolOperation() {
  polyscope::view::moveScale = 0.0;

  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    draggingEvent();
  } else {
    releasedEvent();
  }
}

void SketchInterpolationTool::draggingEvent() {
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
  updateSurfacePoints(xPos, yPos, *getSurfacePointNumPtr());
  disablePointCloud(SurfacePointName);
  registerSurfacePoints(SurfacePointName);

  // Register:
  //  - sketch
  registerSketch(SketchPrefix);
}

void SketchInterpolationTool::releasedEvent() {
  if (getSketchPoints()->size() == 0) return;

  // Find basis points for the surface reconstruction.
  findBasisPoints(true, CLUSTER_MAX_SIZE);
  if (getBasisPointsIndex()->size() == 0) {
    // Disable:
    //  - surface points
    //  - sketch
    disablePointCloud(SurfacePointName);
    disableSketch(SketchPrefix);
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
  std::vector<glm::dvec3> poissonPoints;
  std::vector<glm::dvec3> poissonNormals;
  Surface poissonSurface(&basisPoints, &basisNormals);
  std::tie(poissonPoints, poissonNormals) = poissonSurface.reconstructPoissonSurface(
    "Poisson Interpolation",
    false
  );
  if (poissonPoints.size() == 0) {
    // Disable:
    //  - surface points
    //  - sketch
    disablePointCloud(SurfacePointName);
    disableSketch(SketchPrefix);
    // Reset all member variables.
    resetSketch();
    std::cout << "WARNING: No mesh was reconstructed with Poisson Surface Reconstruction." << std::endl;
    return;
  }

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
  //  - interpolated points
  renderInterpolatedPoints(voxelFilteredPoints, voxelFilteredNormals);

  // Disable:
  //  - surface points
  //  - sketch
  disablePointCloud(SurfacePointName);
  disableSketch(SketchPrefix);
  
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
void SketchInterpolationTool::renderInterpolatedPoints(
  std::vector<glm::dvec3> &newV,
  std::vector<glm::dvec3> &newN
) {
  polyscope::PointCloud* interpolatedCloud = polyscope::registerPointCloud("Interpolation::Interpolated", newV);
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

// Filter the interpolated points with depth
//  1. Cast interpolated points onto the screen plane.
//  2. Search for a candidate point for each discretized grid.
//    - Only points that their normals are directed to cameraOrig.
//  3. Detect depth with DBSCAN
//
//  - surfacePoints:  Positions of the interpolated surface points
//  - surfaceNormals: Normals of the interpolated surface points
std::set<int> SketchInterpolationTool::filterWithDepth(
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

  // Construct map from grid to the closest point's index
  const double castedAverageDist = calcCastedAverageDist();
  const glm::dvec3 cameraOrig = getCameraOrig();

  std::map<std::pair<int, int>, int> gridToClosestPointIndex;
  for (int i = 0; i < surfacePointsSize; i++) {
    glm::dvec3 p = pointsInWorldCoord[i];
    glm::dvec3 mapped_p = pointsCastedOntoScreen[i];

    // Compute the grid where mapped_p falls
    int grid_x = std::floor(mapped_p.x/castedAverageDist);
    int grid_y = std::floor(mapped_p.y/castedAverageDist);
    std::pair<int, int> grid = {grid_x, grid_y};

    if (gridToClosestPointIndex.count(grid)) {
      int closestIndex = gridToClosestPointIndex[grid];
      glm::dvec3 closest_p = pointsInWorldCoord[closestIndex];
      if (glm::length(p - cameraOrig) < glm::length(closest_p - cameraOrig)) {
        gridToClosestPointIndex[grid] = i;
      }
    } else {
      gridToClosestPointIndex[grid] = i;
    }   
  }

  // Compute candidate points' index for each discretized grid.
  std::set<int> candidatePointsIndexSet;
  for (auto i: gridToClosestPointIndex) {
    int closestIndex = i.second;
    glm::dvec3 p = pointsInWorldCoord[closestIndex];
    glm::dvec3 pn = normalsInWorldCoord[closestIndex];
    glm::dvec3 mapped_p = pointsCastedOntoScreen[closestIndex];

    if (!insideSketch(mapped_p.x, mapped_p.y)) continue;          // Inside or outside the sketch
    if (!insideBasisConvexHull(mapped_p.x, mapped_p.y)) continue; // Inside or outside the convex-hull of basis points
    if (glm::dot(pn, p - cameraOrig) >= 0.0) continue;            // Direction of normal vector

    candidatePointsIndexSet.insert(closestIndex); 
  }

  // Detect depth with DBSCAN
  Clustering clustering(&candidatePointsIndexSet, &pointsInWorldCoord, "surface");
  std::set<int> clusteredPointsIndex = clustering.executeClustering(
    DBSCAN_SearchRange*getPointCloud()->getAverageDistance(),
    DBSCAN_MinPoints,
    CLUSTER_MIN_DEPTH
  );

  return clusteredPointsIndex;
}