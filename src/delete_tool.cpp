#include "polyscope/view.h"

#include "delete_tool.hpp"
#include "constants.hpp"

void DeleteTool::launchToolOperation() {
  polyscope::view::moveScale = 0.0;

  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    draggingEvent();
  } else {
    releasedEvent();
  }
}

void DeleteTool::draggingEvent() {
  ImGuiIO &io = ImGui::GetIO();
  ImVec2 mousePos = ImGui::GetMousePos();
  double xPos = io.DisplayFramebufferScale.x * mousePos.x;
  double yPos = io.DisplayFramebufferScale.y * mousePos.y;

  // Cast a ray
  Ray ray(xPos, yPos);
  Ray::Hit hitInfo = ray.castPointToPlane(getScreen());
  if (hitInfo.hit) addSketchPoint(hitInfo.pos);

  // Register sketchPoints as curve network (LINE)
  registerSketchPointsAsCurveNetworkLine(SketchPrefix);
}

void DeleteTool::releasedEvent() {
  if (getSketchPoints()->size() == 0) return;

  // Find basis points for the surface reconstruction.
  findAllBasisPoints();
  if (getBasisPointsIndex()->size() == 0) {
    std::cout << "WARNING: No basis point was found." << std::endl;
    removeCurveNetworkLine(SketchPrefix);
    resetSketch();
    return;
  }

  // Register:
  //  - basis points
  registerBasisPointsAsPointCloud("Basis Points");

  // Remove:
  //  - sketch
  removeCurveNetworkLine(SketchPrefix);

  // 1. Delete basis points from point cloud
  // 2. Update point cloud
  //    - update environments
  //    - update octree
  //    - render points and normals
  getPointCloud()->deletePoints(*getBasisPointsIndex());
  getPointCloud()->updatePointCloud(true);

  // Reset all member variables
  resetSketch();
}