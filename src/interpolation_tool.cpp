#include "polyscope/polyscope.h"

#include "constants.hpp"
#include "interpolation_tool.hpp"
#include "rbf.hpp"
#include "surface.hpp"

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
    addBasisPoints(hitInfo);

    // Register sketch as curve network (LINE)
    registerSketchAsCurveNetworkLine(TracePrefix);
  }

  if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && getBasisPoints()->size() > 0) {
    // Cast basis points to the plane orthogonal to camera direction.
    castBasisToCameraPlane();

    // Discretize the bounded area.
    discretizeCastedBasis();

    // Remove sketch as curve network (LINE)
    removeSketchAsCurveNetworkLine(TracePrefix);

    RBF rbf(
      getAverageDepth(),
      getOrthogonalBasis(),
      getBasisPoints(),
      getBasisOnPlane(),
      getDiscretizedPoints()
    );
    rbf.calcInterpolateSurface();
    Eigen::MatrixXd newV = rbf.castPointsToSurface();
    Eigen::MatrixXd newN;

    // Smooth the points with MLS
    std::tie(newV, newN) = mlsSmoothing(newV);

    getPointCloud()->addPoints(newV, newN);

    resetSketch();
  }
}