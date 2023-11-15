#include "polyscope/polyscope.h"
#include "polyscope/curve_network.h"

#include <glm/gtx/hash.hpp>

#include "constants.hpp"
#include "sketch_tool.hpp"

// Register/Remove sketch as curve network line with sketchName.
// Be aware that the curve network with 
// the same name is overwritten
void SketchTool::registerSketchAsCurveNetworkLine(std::string sketchName) {
  std::vector<std::vector<double>> sketch;
  for (int i = 0; i < boundaryPoints.size(); i++) {
    sketch.push_back({
      boundaryPoints[i].pos.x,
      boundaryPoints[i].pos.y,
      boundaryPoints[i].pos.z
    });
  }
  if (boundaryPoints.size() == 1) {
    sketch.push_back({
      boundaryPoints[0].pos.x,
      boundaryPoints[0].pos.y,
      boundaryPoints[0].pos.z
    });
  };

  polyscope::CurveNetwork* curveNetwork = polyscope::registerCurveNetworkLine(sketchName, sketch);
  curveNetwork->setColor(CurveNetworkColor);
  curveNetwork->setRadius(CurveNetworkRadius);  
}
void SketchTool::removeSketchAsCurveNetworkLine(std::string sketchName) {
  polyscope::removeCurveNetwork(sketchName, false);
}

// Register/Remove sketch as curve network loop with sketchName.
// Be aware that the curve network with 
// the same name is overwritten
void SketchTool::registerSketchAsCurveNetworkLoop(std::string sketchName) {
  std::vector<std::vector<double>> sketch;
  for (int i = 0; i < boundaryPoints.size(); i++) {
    sketch.push_back({
      boundaryPoints[i].pos.x,
      boundaryPoints[i].pos.y,
      boundaryPoints[i].pos.z
    });
  }

  polyscope::CurveNetwork* curveNetwork = polyscope::registerCurveNetworkLoop(sketchName, sketch);
  curveNetwork->setColor(CurveNetworkColor);
  curveNetwork->setRadius(CurveNetworkRadius);
}
void SketchTool::removeSketchAsCurveNetworkLoop(std::string sketchName) {
  polyscope::removeCurveNetwork(sketchName, false);
}

// Select the boundary points the user selected
// and update boundaryPoints.
bool SketchTool::addBoundaryPoints(Hit hitInfo) {
  // if (boundarySet.count(hitInfo.pos)) return false;
  if (boundaryPoints.size() > 0) {
    if (std::abs(hitInfo.depth - averageDepth) >= pointCloud->averageDistance * depthInterval) return false;
  }

  averageDepth = (averageDepth*boundaryPoints.size() + hitInfo.depth) / (boundaryPoints.size() + 1);
  // boundarySet.insert(hitInfo.pos);
  boundaryPoints.push_back(hitInfo);

  return true;
}

// Cast the boundary points to the plane orthogonal to camera direction
// and update boundaryOnPlane.
void SketchTool::castBoundaryToCameraPlane() {

}

// Cast the boundary points to the screen and update boundaryOnPlane.
void SketchTool::castBoundaryToScreen() {

}

// Discretize the boundary and update discretiedPoints.
void SketchTool::discretizeCastedBoundary() {

}

// Reset all member variables.
void SketchTool::resetSketch() {
  *currentMode = 0;
  averageDepth = 0.0;
  // boundarySet.clear();
  boundaryPoints.clear();
  boundaryOnPlane.clear();
  discretizedPoints.clear();
}

// Return the size of boundaryPoints.
int SketchTool::getBoundarySize() {
  return boundaryPoints.size();
}

// Return the pointer to pointCloud.
PointCloud* SketchTool::getPointCloud() {
  return pointCloud;
}
