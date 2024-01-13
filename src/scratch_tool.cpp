#include "polyscope/view.h"

#include "scratch_tool.hpp"
#include "constants.hpp"

void ScratchTool::launchToolOperation() {
  polyscope::view::moveScale = 0.0;

  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    draggingEvent();
  } else {
    releasedEvent();
  }
}

void ScratchTool::draggingEvent() {
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
}

void ScratchTool::releasedEvent() {
  if (getSketchPoints()->size() == 0) return;

  // 1. Delete basis points from point cloud
  // 2. Update point cloud
  //    - update environments
  //    - update octree
  //    - render points and normals
  getPointCloud()->deletePoints(*getSurfacePointsIndex());
  getPointCloud()->updatePointCloud(true);

  // Disable:
  //  - surface points
  disablePointCloud(SurfacePointName);

  // Reset all member variables
  resetSketch();
}