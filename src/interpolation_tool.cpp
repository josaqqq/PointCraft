#include "polyscope/polyscope.h"

#include "polyscope/point_cloud.h"
#include "polyscope/view.h"

#include "constants.hpp"
#include "interpolation_tool.hpp"
#include "rbf.hpp"
#include "surface.hpp"

void visualizeInterpolation(Eigen::MatrixXd &rbfPoints, Eigen::MatrixXd &mlsPoints) {
  // RBF points
  std::vector<std::vector<double>> points(rbfPoints.size());
  for (int i = 0; i < rbfPoints.rows(); i++) {
    points[i] = {
      rbfPoints(i, 0),
      rbfPoints(i, 1),
      rbfPoints(i, 2)
    };
  }
  polyscope::PointCloud* rbfCloud = polyscope::registerPointCloud(RBFName, rbfPoints);
  rbfCloud->setPointColor(RBFColor);
  rbfCloud->setPointRadius(PointRadius);
}

void InterpolationTool::drawSketch() {
  ImGuiIO &io = ImGui::GetIO();
  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    ImVec2 mousePos = ImGui::GetMousePos();
    double xPos = io.DisplayFramebufferScale.x * mousePos.x;
    double yPos = io.DisplayFramebufferScale.y * mousePos.y;

    // Cast a ray
    Ray ray(xPos, yPos);
    Hit hitInfo = ray.castPointToPlane(getScreen());
    if (!hitInfo.hit) return;
    addSketchPoint(hitInfo.pos);

    // Register sketchPoints as curve network (LINE)
    registerSketchPointsAsCurveNetworkLine(SketchPrefix);
  }

  if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && getSketchPoints()->size() > 0) {
    // Discretize the bounded area.
    discretizeSketchPoints();

    // Find basis points
    findBasisPoints();
    if (getBasisPointsIndex()->size() == 0) {
      removePointCloud("Basis Points");
      removeCurveNetworkLine(SketchPrefix);
      resetSketch();
      return;
    }

    // Register calculated points.
    // registerDiscretizedPointsAsPontCloud("Discretized Points");
    registerBasisPointsAsPointCloud("Basis Points");

    // Calculate approximate surface with Poisson Surface Reconstruction
    int basisPointsSize = getBasisPointsIndex()->size();
    Eigen::MatrixXd psrPoints(basisPointsSize, 3);
    Eigen::MatrixXd psrNormals(basisPointsSize, 3);
    for (int i = 0; i < basisPointsSize; i++) {
      int idx = (*getBasisPointsIndex())[i];
      psrPoints(i, 0) = getPointCloud()->meshV(idx, 0);
      psrPoints(i, 1) = getPointCloud()->meshV(idx, 1);
      psrPoints(i, 2) = getPointCloud()->meshV(idx, 2);

      psrNormals(i, 0) = getPointCloud()->meshN(idx, 0);
      psrNormals(i, 1) = getPointCloud()->meshN(idx, 1);
      psrNormals(i, 2) = getPointCloud()->meshN(idx, 2);
    }
    poissonReconstruct(
      "Interpolation: PoissonSurfaceReconstruction", 
      getPointCloud()->averageDistance, 
      psrPoints, 
      psrNormals
    );

    // Calculate approximate surface with RBF
    RBF rbf(
      getPointCloud(),
      getScreen(),
      getBasisPointsIndex(),
      getDiscretizedPoints()
    );
    rbf.calcInterpolateSurface();
    Eigen::MatrixXd rbfPoints = rbf.castPointsToSurface();

    // Smooth the points with MLS
    Eigen::MatrixXd mlsPoints;
    Eigen::MatrixXd mlsNormals;
    std::tie(mlsPoints, mlsNormals) = mlsSmoothing(MLSName, rbfPoints);

    // // Visualize interpolation
    // visualizeInterpolation(rbfPoints, mlsPoints);

    // Add the interpolated points
    // getPointCloud()->addPoints(mlsPoints, mlsNormals);

    // Remove sketch as curve network (LINE)
    removeCurveNetworkLine(SketchPrefix);

    resetSketch();
  }
}