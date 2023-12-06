#include "polyscope/polyscope.h"
#include "polyscope/view.h"

#include "mls_spray_tool.hpp"
#include "surface.hpp"
#include "constants.hpp"

void MLSSprayTool::launchToolOperation() {
  polyscope::view::moveScale = 0.0;

  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    draggingEvent();
  } else {
    releasedEvent();
  }
}

void MLSSprayTool::draggingEvent() {
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
  updateSurfacePoints(xPos, yPos, MLS_SprayNearestNeighbors);
  removePointCloud(SurfacePointName);
  registerSurfacePointsAsPointCloud(SurfacePointName);
  // If no point was selected, then return.
  if (getSurfacePointsIndex()->size() == 0) return;

  std::vector<glm::dvec3> surfacePoints;
  std::vector<int>        *surfacePointsIndexPtr = getSurfacePointsIndex();
  std::vector<glm::dvec3> *verticesPtr = getPointCloud()->getVertices();
  for (size_t i = 0; i < surfacePointsIndexPtr->size(); i++) {
    int idx = (*surfacePointsIndexPtr)[i];
    surfacePoints.push_back((*verticesPtr)[idx]);
  }

  // Construct MLS surface and project random points onto the surface
  std::vector<glm::dvec3> dummyNormals;
  std::vector<glm::dvec3> newV, newN;
  Surface mlsSurface(MLSName, &surfacePoints, &dummyNormals);
  std::tie(newV, newN) = mlsSurface.projectMLSSurface(
    xPos,
    yPos,
    getPointCloud()->getBoundingSphereRadius(),
    getPointCloud()->getAverageDistance(),
    MLS_SpraySize
  );

  // Add the interpolated points to the point cloud.
  getPointCloud()->addPoints(newV, newN);
}

void MLSSprayTool::releasedEvent() {
  if (getSketchPoints()->size() == 0) return;

  // Remove surfacePointsIndex
  removePointCloud(SurfacePointName);

  // Reset all member variables.
  resetSketch();
}
