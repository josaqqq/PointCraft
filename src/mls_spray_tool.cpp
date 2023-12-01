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

  // // Cast a ray to pointCloud
  // Ray ray(xPos, yPos);
  // Hit hitInfo = ray.searchNearestNeighbor(getPointCloud()->getAverageDistance());
  // if (!hitInfo.hit) return;

  // // Add the hit point to sketchPoints
  // addSketchPoint(hitInfo.pos);

  // // Search for K-nearest-neighbors
  // glm::dvec3 cameraOrig = getCameraOrig();
  // glm::dvec3 cameraDir = getCameraDir();
  // glm::dvec3 center = cameraOrig + glm::dot(hitInfo.pos - cameraOrig, cameraDir)*cameraDir;
  // Eigen::MatrixXd basisVertices = searchKNeighbors(center, MLS_SprayK);

  // // Construct MLS surface and project random points onto the surface
  // Eigen::MatrixXd dummyNormals;
  // Eigen::MatrixXd newV, newN;
  // Surface mlsSurface(MLSName, basisVertices, dummyNormals);
  // std::tie(newV, newN) = mlsSurface.projectMLSSurface(
  //   getPointCloud()->getAverageDistance(),
  //   MLS_SpraySize
  // );

  // // Display added points as pseudo surface temporarily
  // displayAddedPoints();
}

// Display added points as pseudo surface temporarily
void MLSSprayTool::displayAddedPoints() {

}

void MLSSprayTool::releasedEvent() {
  if (getSketchPoints()->size() == 0) return;

  Eigen::MatrixXd newV;
  Eigen::MatrixXd newN;

  // Add the interpolated points
  getPointCloud()->addPoints(newV, newN);

  // Reset all member variables.
  resetSketch();
}

// Search pointCloud for nearest neighbors.
//  - center: center of search range
//  - K_size: the max number of nearest neighbors
Eigen::MatrixXd MLSSprayTool::searchKNeighbors(glm::dvec3 center, int K_size) {
  Eigen::MatrixXd dummy;
  return dummy;
}