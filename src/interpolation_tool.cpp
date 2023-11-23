#include "polyscope/polyscope.h"

#include "polyscope/point_cloud.h"
#include "polyscope/view.h"

#include "constants.hpp"
#include "interpolation_tool.hpp"
#include "rbf.hpp"
#include "surface.hpp"

void InterpolationTool::drawSketch() {
  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) draggingEvent();
  if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && getSketchPoints()->size() > 0) releasedEvent();
}

void InterpolationTool::draggingEvent() {
  ImGuiIO &io = ImGui::GetIO();
  ImVec2 mousePos = ImGui::GetMousePos();
  double xPos = io.DisplayFramebufferScale.x * mousePos.x;
  double yPos = io.DisplayFramebufferScale.y * mousePos.y;

  // Cast a ray
  Ray ray(xPos, yPos);
  Hit hitInfo = ray.castPointToPlane(getScreen());
  if (hitInfo.hit) addSketchPoint(hitInfo.pos);

  // Register sketchPoints as curve network (LINE)
  registerSketchPointsAsCurveNetworkLine(SketchPrefix);
}

void InterpolationTool::releasedEvent() {
  // Discretize the bounded area.
  discretizeSketchPoints();

  // Find basis points
  findBasisPoints();
  if (getBasisPointsIndex()->size() == 0) {
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
  Eigen::MatrixXi psrFaces;
  for (int i = 0; i < basisPointsSize; i++) {
    int idx = (*getBasisPointsIndex())[i];
    psrPoints(i, 0) = getPointCloud()->Vertices(idx, 0);
    psrPoints(i, 1) = getPointCloud()->Vertices(idx, 1);
    psrPoints(i, 2) = getPointCloud()->Vertices(idx, 2);

    psrNormals(i, 0) = getPointCloud()->Normals(idx, 0);
    psrNormals(i, 1) = getPointCloud()->Normals(idx, 1);
    psrNormals(i, 2) = getPointCloud()->Normals(idx, 2);
  }

  std::tie(psrPoints, psrFaces) = poissonReconstruct(
    "Interpolation: PSR",
    getPointCloud()->getAverageDistance(),
    psrPoints,
    psrNormals  
  );
  if (psrPoints.rows() == 0) {
    removeCurveNetworkLine(SketchPrefix);
    resetSketch();
    return;
  }

  // Cast rays to reconstructed surface
  std::vector<Hit> meshHits;
  glm::dvec3 cameraOrig = polyscope::view::getCameraWorldPosition();
  for (int i = 0; i < getDiscretizedPoints()->size(); i++) {
    glm::dvec3 discretizedPoint = (*getDiscretizedPoints())[i];
    
    // Cast a ray to reconstructed surface
    Ray ray(cameraOrig, discretizedPoint);
    Hit hitInfo = ray.meshIntersection(psrPoints, psrFaces, getPointCloud());
    if (hitInfo.hit) meshHits.push_back(hitInfo);
  }

  // Add the interpolated points
  Eigen::MatrixXd newV(meshHits.size(), 3);
  Eigen::MatrixXd newN(meshHits.size(), 3);
  for (int i = 0; i < meshHits.size(); i++) {
    Hit hitInfo = meshHits[i];

    newV.row(i) << 
      hitInfo.pos.x,
      hitInfo.pos.y,
      hitInfo.pos.z;
    newN.row(i) << 
      hitInfo.normal.x,
      hitInfo.normal.y,
      hitInfo.normal.z;
  }
  getPointCloud()->addPoints(newV, newN);

  // Register Interpolated Points as point cloud
  renderInterpolatedPoints(newV, newN);

  // Remove sketch as curve network (LINE)
  removeCurveNetworkLine(SketchPrefix);

  resetSketch();
}

void InterpolationTool::renderInterpolatedPoints(
  Eigen::MatrixXd &newV, 
  Eigen::MatrixXd &newN
) {
  polyscope::PointCloud* interpolatedCloud = polyscope::registerPointCloud("Interpolated Points", newV);
  interpolatedCloud->setPointColor(glm::dvec3(1.0, 0.0, 0.0));
  interpolatedCloud->setPointRadius(BasisPointRadius);

  polyscope::PointCloudVectorQuantity* interpolatedVectorQuantity = interpolatedCloud->addVectorQuantity(NormalName, newN);
  interpolatedVectorQuantity->setVectorColor(glm::dvec3(1.0, 0.0, 0.0));
  interpolatedVectorQuantity->setVectorLengthScale(NormalLength);
  interpolatedVectorQuantity->setVectorRadius(NormalRadius * 1.1);
  interpolatedVectorQuantity->setEnabled(NormalEnabled);
  interpolatedVectorQuantity->setMaterial(NormalMaterial);
}