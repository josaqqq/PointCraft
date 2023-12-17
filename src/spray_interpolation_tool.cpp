#include "polyscope/polyscope.h"
#include "polyscope/view.h"

#include <fstream>

#include "spray_interpolation_tool.hpp"
#include "cluster.hpp"
#include "surface.hpp"
#include "constants.hpp"

void SprayInterpolationTool::launchToolOperation() {
  polyscope::view::moveScale = 0.0;

  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    draggingEvent();
  } else {
    releasedEvent();
  }
}

void SprayInterpolationTool::exportLog(std::string logFileName) {
  // Open Log File
  std::ofstream logFile(logFileName, std::ios::app);

  logFile << "\nSpray Interpolation Tool Log:\n";

  // close Log File
  logFile.close();
}

void SprayInterpolationTool::draggingEvent() {
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
  resetSurfacePointsIndex();
  updateSurfacePoints(xPos, yPos, *getSurfacePointNumPtr());
  removePointCloud(SurfacePointName);
  registerSurfacePointsAsPointCloud(SurfacePointName);

  // Fetch actual points' positions from the point cloud
  // If no point was selected, then return.
  if (getSurfacePointsIndex()->size() == 0) return;
  std::vector<glm::dvec3> surfacePoints;
  std::set<int>           *surfacePointsIndexPtr = getSurfacePointsIndex();
  std::vector<glm::dvec3> *verticesPtr = getPointCloud()->getVertices();
  for (int idx: *surfacePointsIndexPtr) {
    surfacePoints.push_back((*verticesPtr)[idx]);
  }

  // Construct MLS surface and project random points onto the surface
  std::vector<glm::dvec3> dummyNormals;
  std::vector<glm::dvec3> mlsVertices, mlsNormals;
  Surface mlsSurface(&surfacePoints, &dummyNormals);
  std::tie(mlsVertices, mlsNormals) = mlsSurface.projectMLSSurface(
    MLSName,
    glm::dvec2(xPos, yPos),
    getPointCloud()->getBoundingBoxSide(),
    getPointCloud()->getAverageDistance(),
    MLS_SpraySize
  );
  assert(mlsVertices.size() == MLS_SpraySize);
  assert(mlsNormals.size() == mlsVertices.size());

  // Filter the reconstructed surface with DBSCAN
  std::set<int> candidatePointsIndexSet;
  for (size_t i = 0; i < surfacePoints.size() + MLS_SpraySize; i++) {
    candidatePointsIndexSet.insert(i);

    // Inside mlsVertices and mlsNormals 
    //  -> <mlsVertices>-<surfacePoints>
    //  -> <mlsNormals>-<dummy normals>
    if (i < surfacePoints.size()) {
      mlsVertices.push_back(surfacePoints[i]);
      mlsNormals.push_back(glm::dvec3(0.0, 0.0, 0.0));
    }
  }

  Clustering clustering(&candidatePointsIndexSet, &mlsVertices, "surface");
  std::set<int> clusteredPointsIndex = clustering.executeClustering(
    DBSCAN_SearchRange*getPointCloud()->getAverageDistance(),
    DBSCAN_MinPoints,
    CLUSTER_MAX_SIZE
  );
  std::vector<glm::dvec3> clusteredPoints, clusteredNormals;
  for (int idx: clusteredPointsIndex) {
    if (idx >= MLS_SpraySize) continue;

    clusteredPoints.push_back(mlsVertices[idx]);
    clusteredNormals.push_back(mlsNormals[idx]);
  }

  // Filter the reconstructed surface with voxel
  std::set<int> voxelFilteredIndex = filterWithVoxel(clusteredPoints);
  std::vector<glm::dvec3> voxelFilteredPoints, voxelFilteredNormals;
  for (int idx: voxelFilteredIndex) {
    voxelFilteredPoints.push_back(clusteredPoints[idx]);
    voxelFilteredNormals.push_back(clusteredNormals[idx]);
  }

  // Add the interpolated points.
  getPointCloud()->addPoints(voxelFilteredPoints, voxelFilteredNormals);
}

void SprayInterpolationTool::releasedEvent() {
  if (getSketchPoints()->size() == 0) return;

  // Remove:
  //  - surface points
  removePointCloud(SurfacePointName);
  
  // Update point cloud
  //    - update environments
  //    - update octree
  //    - render points and normals
  getPointCloud()->updatePointCloud(true);

  // Reset all member variables.
  resetSketch();
}
