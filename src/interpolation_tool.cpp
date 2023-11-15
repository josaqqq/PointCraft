#include "polyscope/polyscope.h"

#include "constants.hpp"
#include "interpolation_tool.hpp"
#include "rbf.hpp"

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

    // Register sketch as curve network (LINE)
    registerSketchAsCurveNetworkLine(TracePrefix);
  }

  if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && getBoundaryPoints()->size() > 0) {
    // Cast boundary points to the plane orthogonal to camera direction.
    castBoundaryToCameraPlane();

    // Discretize the bounded area.
    discretizeCastedBoundary();

    // Remove sketch as curve network (LINE)
    removeSketchAsCurveNetworkLine(TracePrefix);

    RBF rbf(
      getAverageDepth(),
      getOrthogonalBasis(),
      getBoundaryPoints(),
      getBoundaryOnPlane(),
      getDiscretizedPoints()
    );
    rbf.calcInterpolateSurface();
    Eigen::MatrixXd newV = rbf.castPointsToSurface();

    // TODO: We need to calculate normals
    Eigen::MatrixXd newN(newV.rows(), 3);
    for (int i = 0; i < newV.rows(); i++) {
      double length = glm::length(glm::dvec3(newV(i, 0), newV(i, 1), newV(i, 2)));
      newN(i, 0) = newV(i, 0) / length;
      newN(i, 1) = newV(i, 1) / length;
      newN(i, 2) = newV(i, 2) / length;
    }

    // Register calculated point cloud
    registerPatchAsPointCloud(newV, newN);

    getPointCloud()->addPoints(newV, newN);

    resetSketch();
  }
}