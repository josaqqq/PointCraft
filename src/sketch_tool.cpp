#include "polyscope/polyscope.h"

#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"

#include <glm/gtx/hash.hpp>

#include "constants.hpp"
#include "sketch_tool.hpp"

// Register/Remove Patch as point cloud with patchName.
// Be aware that the point cloud with 
// the same name is overwritten.
void SketchTool::registerPatchAsPointCloud(std::string patchName) {
  // TODO: Implemented for debug
  std::vector<std::vector<double>> patch;
  for (int i = 0; i < basisOnPlane.size(); i++) {
    patch.push_back({
      basisOnPlane[i].x,
      basisOnPlane[i].y,
      0.0d
    });
  }
  for (int i = 0; i < discretizedPoints.size(); i++) {
    patch.push_back({
      discretizedPoints[i].x,
      discretizedPoints[i].y,
      0.0d
    });
  }

  polyscope::PointCloud* patchCloud = polyscope::registerPointCloud(patchName, patch);
  patchCloud->setPointColor(PointColor);
  patchCloud->setPointRadius(PointRadius);
}
void SketchTool::registerPatchAsPointCloud(std::string patchName, Eigen::MatrixXd vertices, Eigen::MatrixXd normals) {
  // TODO: Implemented for debug
  polyscope::PointCloud* points = polyscope::registerPointCloud(patchName, vertices);
  points->setPointColor(PointColor);
  points->setPointRadius(PointRadius);

  polyscope::PointCloudVectorQuantity *vectorQuantity = points->addVectorQuantity(NormalName, normals);
  vectorQuantity->setVectorColor(NormalColor);
  vectorQuantity->setVectorLengthScale(NormalLength);
  vectorQuantity->setVectorRadius(NormalRadius);
  vectorQuantity->setEnabled(NormalEnabled);
}
void SketchTool::removePatchAsPointCloud(std::string patchName) {
  polyscope::removePointCloud(patchName, false);
}

// Register/Remove sketch as curve network line with sketchName.
// Be aware that the curve network with 
// the same name is overwritten
void SketchTool::registerSketchAsCurveNetworkLine(std::string sketchName) {
  std::vector<std::vector<double>> sketch;
  for (int i = 0; i < basisPoints.size(); i++) {
    sketch.push_back({
      basisPoints[i].pos.x,
      basisPoints[i].pos.y,
      basisPoints[i].pos.z
    });
  }
  if (basisPoints.size() == 1) {
    sketch.push_back({
      basisPoints[0].pos.x,
      basisPoints[0].pos.y,
      basisPoints[0].pos.z
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
  for (int i = 0; i < basisPoints.size(); i++) {
    sketch.push_back({
      basisPoints[i].pos.x,
      basisPoints[i].pos.y,
      basisPoints[i].pos.z
    });
  }

  polyscope::CurveNetwork* curveNetwork = polyscope::registerCurveNetworkLoop(sketchName, sketch);
  curveNetwork->setColor(CurveNetworkColor);
  curveNetwork->setRadius(CurveNetworkRadius);
}
void SketchTool::registerSketchAsCurveNetworkLoop(std::string sketchName, Eigen::MatrixXd vertices) {
  polyscope::CurveNetwork* curveNetwork = polyscope::registerCurveNetworkLoop(sketchName, vertices);
  curveNetwork->setColor(CurveNetworkColor);
  curveNetwork->setRadius(CurveNetworkRadius);
}
void SketchTool::removeSketchAsCurveNetworkLoop(std::string sketchName) {
  polyscope::removeCurveNetwork(sketchName, false);
}

// Select the basis points the user selected
// and update basisPoints.
bool SketchTool::addBasisPoints(Hit hitInfo) {
  // if (basisSet.count(hitInfo.pos)) return false;
  if (basisPoints.size() > 0) {
    if (std::abs(hitInfo.depth - averageDepth) >= pointCloud->averageDistance * depthInterval) return false;
  }

  averageDepth = (averageDepth*basisPoints.size() + hitInfo.depth) / (basisPoints.size() + 1);
  // basisSet.insert(hitInfo.pos);
  basisPoints.push_back(hitInfo);

  return true;
}

// Cast the basis points to the plane orthogonal to camera direction
// and update basisOnPlane.
void SketchTool::castBasisToCameraPlane() {
  const glm::dvec3 orig       = polyscope::view::getCameraWorldPosition();
  const glm::dvec3 cameraDir  = polyscope::view::screenCoordsToWorldRay(glm::vec2(polyscope::view::windowWidth/2, polyscope::view::windowHeight/2));

  // calculate orthogonal basis and then set the values to u, v.
  calcOrthogonalBasis(cameraDir);

  const double h = glm::dot(orig, cameraDir);
  for (int i = 0; i < basisPoints.size(); i++) {
    glm::dvec3 point = basisPoints[i].pos;
    glm::dvec3 castedPoint = point - (glm::dot(point, cameraDir) - h)*cameraDir;

    basisOnPlane.push_back(glm::vec2(
      glm::dot(orthoU, castedPoint - orig),
      glm::dot(orthoV, castedPoint - orig)
    ));
  }
}

// Cast the basis points to the screen and update basisOnPlane.
void SketchTool::castBasisToScreen() {

}

// Discretize the basis and update discretiedPoints.
void SketchTool::discretizeCastedBasis() {
  const int polygonSize = basisOnPlane.size();

  // Comput the search basis.
  const double INF = 100000.0;
  double min_x = INF, max_x = -INF;
  double min_y = INF, max_y = -INF;
  for (int i = 0; i < polygonSize; i++) {
    min_x = std::min(min_x, basisOnPlane[i].x);
    max_x = std::max(max_x, basisOnPlane[i].x);
    min_y = std::min(min_y, basisOnPlane[i].y);
    max_y = std::max(max_y, basisOnPlane[i].y);
  }

  for (double i = min_x; i < max_x; i += pointCloud->averageDistance) {
    for (double j = min_y; j < max_y; j += pointCloud->averageDistance) {
      if (!insidePolygon(i, j, polygonSize)) continue;
      discretizedPoints.push_back(glm::vec2(i, j));
    }
  }
}

// Reset all member variables.
void SketchTool::resetSketch() {
  *currentMode = 0;
  averageDepth = 0.0;
  // basisSet.clear();
  basisPoints.clear();
  basisOnPlane.clear();
  discretizedPoints.clear();
}

// Return the pointer to member variables.
double SketchTool::getAverageDepth() {
  return averageDepth;
}
PointCloud* SketchTool::getPointCloud() {
  return pointCloud;
}
std::vector<Hit>* SketchTool::getBasisPoints() {
  return &basisPoints;
}
std::vector<glm::dvec2>* SketchTool::getBasisOnPlane() {
  return &basisOnPlane;
}
std::vector<glm::dvec2>* SketchTool::getDiscretizedPoints() {
  return &discretizedPoints;
}
std::pair<glm::dvec3, glm::dvec3> SketchTool::getOrthogonalBasis() {
  return { orthoU, orthoV };
}

// Calculate orthogonal basis with Gram-Schmidt orthonormalization.
void SketchTool::calcOrthogonalBasis(glm::dvec3 normal) {
  glm::dvec3 e1 = glm::dvec3(1.0, 0.0, 0.0);
  glm::dvec3 e2 = glm::dvec3(0.0, 1.0, 0.0);
  if (e1 == normal) e1 = glm::dvec3(0.0, 0.0, 1.0);
  if (e2 == normal) e2 = glm::dvec3(0.0, 0.0, 1.0);

  orthoU = e1 - glm::dot(e1, normal)*normal;
  orthoU /= glm::length(orthoU);

  orthoV = e2 - glm::dot(e2, normal)*normal - glm::dot(e2, orthoU)*orthoU;
  orthoV /= glm::length(orthoV);
}

// Check the inside/outside of the polygon.
//  1.  Draw a half-line parallel to the x-axis from a point.
//  2.  Determine that if there are an odd number of intersections
//      between this half-line and the polygon, it is inside, and if 
//      there are an even number, it is outside.
bool SketchTool::insidePolygon(double x, double y, const int polygonSiz) {
  const double EPS = 1e-5; 

  int crossCount = 0;
  for (int i = 0; i < polygonSiz; i++) {
    glm::dvec2 u = glm::dvec2(basisOnPlane[i].x, basisOnPlane[i].y);
    glm::dvec2 v = glm::dvec2(basisOnPlane[(i + 1) % polygonSiz].x, basisOnPlane[(i + 1) % polygonSiz].y);

    // If u, v are too close to (x, y), then offset. 
    if (std::abs(u.y - y) < EPS) {
      if (u.y - y >= 0.0d)  u.y += EPS;
      else u.y -= EPS;
    }
    if (std::abs(v.y - y) < EPS) {
      if (v.y - y >= 0.0d)  v.y += EPS;
      else v.y -= EPS;
    }

    // If (u, v) is parallel to x-axis, then skip it.
    if (std::abs(u.y - y) < EPS && std::abs(v.y - y) < EPS) continue;

    // If u, v are in the same side of the half-line, then skip it.
    if ((u.y - y)*(v.y - y) >= 0.0d) continue;

    // If (u, v) doesn't intersect the half-line, then skip it.
    double crossX = u.x + (v.x - u.x)*std::abs(y - u.y)/std::abs(v.y - u.y);
    if (crossX < x) continue;

    crossCount++;
  }

  if (crossCount% 2 == 0) return false;
  else return true;
}