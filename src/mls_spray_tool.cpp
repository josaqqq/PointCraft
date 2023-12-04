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

  // Update surfacePoints
  updateSurfacePoints(xPos, yPos, MLS_SprayNearestNeighbors);
  removePointCloud(SurfacePointName);
  registerSurfacePointsAsPointCloud(SurfacePointName);

  // If no point was selected, then return.
  if (getSurfacePoints()->size() == 0) return;

  // Construct MLS surface and project random points onto the surface
  std::vector<glm::dvec3> dummyNormals;
  std::vector<glm::dvec3> newV, newN;
  Surface mlsSurface(MLSName, getSurfacePoints(), &dummyNormals);
  std::tie(newV, newN) = mlsSurface.projectMLSSurface(
    xPos,
    yPos,
    getPointCloud()->getAverageDistance(),
    MLS_SpraySize
  );

  // Add the interpolated points to the point cloud.
  getPointCloud()->addPoints(newV, newN);
}

void MLSSprayTool::releasedEvent() {
  if (getSketchPoints()->size() == 0) return;

  // Remove surfacePoints
  removePointCloud(SurfacePointName);

  // Reset all member variables.
  resetSketch();
}
