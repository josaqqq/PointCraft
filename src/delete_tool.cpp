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
  findAllBasisPoints(false);
  if (getBasisPointsIndex()->size() == 0) {
    std::cout << "WARNING: No basis point was found." << std::endl;
    removeCurveNetworkLine(SketchPrefix);
    resetSketch();
    return;
  }

  // Register calculated points.
  registerBasisPointsAsPointCloud("Basis Points");

  // Remove sketch as curve network (LINE)
  removeCurveNetworkLine(SketchPrefix);

  // Delete basis points from point cloud
  getPointCloud()->deletePoints(*getBasisPointsIndex());

  // Reset all member variables
  resetSketch();
}