#include "polyscope/polyscope.h"

#include "constants.hpp"
#include "interpolation_tool.hpp"

void InterpolationTool::drawSketch() {
  ImGuiIO &io = ImGui::GetIO();
  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    ImVec2 mousePos = ImGui::GetMousePos();
    double xPos = io.DisplayFramebufferScale.x * mousePos.x;
    double yPos = io.DisplayFramebufferScale.y * mousePos.y;

    // Cast a ray
    Ray ray(xPos, yPos, getPointCloud());
    Hit hitInfo = ray.searchNeighborPoints(CurveNetworkRadius);
    if (!hitInfo.hit) return;
    addBoundaryPoints(hitInfo);

    std::cout << hitInfo.pos.x << " " << hitInfo.pos.y << " " << hitInfo.pos.z << std::endl;

    // Register sketch as curve network (LINE)
    registerSketchAsCurveNetworkLine(TracePrefix);
  }

  if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && getBoundarySize() > 0) {
    // Cast boundary points to the plane orthogonal to camera direction
    castBoundaryToCameraPlane();

    resetSketch();
  }
}