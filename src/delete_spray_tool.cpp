#include "polyscope/view.h"

#include "delete_spray_tool.hpp"
#include "ray.hpp"
#include "constants.hpp"

void DeleteSprayTool::launchToolOperation() {
  polyscope::view::moveScale = 0.0;

  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    draggingEvent();
  } else {
    releasedEvent();
  }
}

void DeleteSprayTool::draggingEvent() {
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
  updateSurfacePoints(xPos, yPos, 1);
  removePointCloud(SurfacePointName);
  registerSurfacePointsAsPointCloud(SurfacePointName);
  // If no point was selected, then return.
  if (getSurfacePointsIndex()->size() == 0) return;

  // Delete the selected surface points from the point cloud
  getPointCloud()->deletePoints(*getSurfacePointsIndex());
}

void DeleteSprayTool::releasedEvent() {
  if (getSketchPoints()->size() == 0) return;
  
  // Remove surfacePointsIndex
  removePointCloud(SurfacePointName);

  // Reset all member variables
  resetSketch();
}