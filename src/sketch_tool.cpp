#include "polyscope/polyscope.h"

#include "polyscope/view.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"

#include <glm/gtx/hash.hpp>

#include "sketch_tool.hpp"
#include "constants.hpp"
#include "cluster.hpp"

/*
  Manage functions
*/

// Initialize screen information
void SketchTool::initSketch() {
  screenDist = polyscope::view::nearClipRatio*polyscope::state::lengthScale * ScreenOffset;
  cameraOrig = polyscope::view::getCameraWorldPosition();
  cameraDir = polyscope::view::screenCoordsToWorldRay(
    glm::vec2(polyscope::view::windowWidth/2, polyscope::view::windowHeight/2)
  );
  glm::dvec3 screenOrig = cameraOrig + screenDist*cameraDir;

  // Plane on screenDist
  screen = Plane(screenOrig, cameraDir);
}

// Reset all member variables.
void SketchTool::resetSketch() {
  *currentMode = 0;

  averageDepth = 0.0;

  screenDist = 0.0;
  cameraOrig = glm::dvec3(0.0, 0.0, 0.0);
  cameraDir = glm::dvec3(0.0, 0.0, 0.0);
  screen = Plane();

  sketchPoints.clear();
  basisPointsIndex.clear();
}

/*
  Viewer functions
*/
void SketchTool::registerBasisPointsAsPointCloud(std::string name) {
  // Show basisPoints

  std::vector<std::vector<double>> points;
  std::vector<std::vector<double>> normals;
  for (int i = 0; i < basisPointsIndex.size(); i++) {
    int idx = basisPointsIndex[i];

    points.push_back({
      pointCloud->Vertices(idx, 0),
      pointCloud->Vertices(idx, 1),
      pointCloud->Vertices(idx, 2)
    });
    normals.push_back({
      pointCloud->Normals(idx, 0),
      pointCloud->Normals(idx, 1),
      pointCloud->Normals(idx, 2)
    });
  }

  polyscope::PointCloud* patchCloud = polyscope::registerPointCloud(name, points);
  patchCloud->setPointColor(BasisPointColor);
  patchCloud->setPointRadius(BasisPointRadius);
  patchCloud->setEnabled(BasisPointEnabled);

  polyscope::PointCloudVectorQuantity* patchVectorQuantity = patchCloud->addVectorQuantity(NormalName, normals);
  patchVectorQuantity->setVectorColor(BasisPointColor);
  patchVectorQuantity->setVectorLengthScale(NormalLength);
  patchVectorQuantity->setVectorRadius(NormalRadius * 1.1);
  patchVectorQuantity->setEnabled(NormalEnabled);
  patchVectorQuantity->setMaterial(NormalMaterial);
}
void SketchTool::removePointCloud(std::string name) {
  polyscope::removePointCloud(name, false);
}

// Register/Remove curve network line with name.
// Be aware that the curve network with 
// the same name is overwritten
void SketchTool::registerSketchPointsAsCurveNetworkLine(std::string name) {
  std::vector<std::vector<double>> sketch;
  for (int i = 0; i < sketchPoints.size(); i++) {
    sketch.push_back({
      sketchPoints[i].x,
      sketchPoints[i].y,
      sketchPoints[i].z
    });
  }
  if (sketchPoints.size() == 1) {
    sketch.push_back({
      sketchPoints[0].x,
      sketchPoints[0].y,
      sketchPoints[0].z
    });
  };

  polyscope::CurveNetwork* curveNetwork = polyscope::registerCurveNetworkLine(name, sketch);
  curveNetwork->setColor(CurveNetworkColor);
  curveNetwork->setRadius(CurveNetworkRadius);  
}
void SketchTool::removeCurveNetworkLine(std::string name) {
  polyscope::removeCurveNetwork(name, false);
}

// Register/Remove curve network loop with name.
// Be aware that the curve network with 
// the same name is overwritten
void SketchTool::registerSketchPointsAsCurveNetworkLoop(std::string name) {
  std::vector<std::vector<double>> sketch;
  for (int i = 0; i < sketchPoints.size(); i++) {
    sketch.push_back({
      sketchPoints[i].x,
      sketchPoints[i].y,
      sketchPoints[i].z
    });
  }

  polyscope::CurveNetwork* curveNetwork = polyscope::registerCurveNetworkLoop(name, sketch);
  curveNetwork->setColor(CurveNetworkColor);
  curveNetwork->setRadius(CurveNetworkRadius);
}
void SketchTool::removeCurveNetworkLoop(std::string name) {
  polyscope::removeCurveNetwork(name, false);
}

/*
  Geometry functions
*/

// Cast the specified point to screen.
void SketchTool::addSketchPoint(glm::dvec3 p) {
  sketchPoints.push_back(p);
}

// Find basis points.
//  - If extendedSearch is true, extend the sketched area.
//  - Cast all points of the point cloud onto the screen plane.
//  - Judge inside/outside of the sketch.
//  - Check whether the normal and the camera direction are faced each other.
void SketchTool::findBasisPoints(bool extendedSearch) {
  // If extendedSearch is true, extend the sketched area.
  extendSketchedArea();

  // Cast all points of the point cloud onto the screen plane.
  std::vector<glm::dvec3> pointsCastedOntoScreen;
  for (int i = 0; i < pointCloud->Vertices.rows(); i++) {
    glm::dvec3 p = glm::dvec3(
      pointCloud->Vertices(i, 0),
      pointCloud->Vertices(i, 1),
      pointCloud->Vertices(i, 2)
    );

    // Cast a ray from p to cameraOrig onto screen.
    Ray ray(p, cameraOrig);
    Hit hitInfo = ray.castPointToPlane(&screen);
    if (hitInfo.hit) {
      glm::dvec3 castedP = screen.mapCoordinates(hitInfo.pos);
      pointsCastedOntoScreen.push_back(castedP);
    }
  }

  // Judge inside/outside of the sketch
  // and check the normal of the points.
  std::vector<int> candidatePointsIndex;
  for (int i = 0; i < pointsCastedOntoScreen.size(); i++) {
    glm::dvec3 p = pointsCastedOntoScreen[i];
    glm::dvec3 n = glm::dvec3(
      pointCloud->Normals(i, 0),
      pointCloud->Normals(i, 1),
      pointCloud->Normals(i, 2)
    );

    if (insideSketch(p.x, p.y) && glm::dot(cameraDir, n) < 0.0) {
      candidatePointsIndex.push_back(i);
    }
  }

  // Depth detection with DBSCAN
  Clustering clustering(&candidatePointsIndex, pointCloud, &screen);
  basisPointsIndex = clustering.executeDBSCAN(DBSCAN_SearchRange*pointCloud->getAverageDistance(), DBSCAN_MinPoints);
}

// Check the inside/outside of the polygon.
//  1.  Draw a half-line parallel to the x-axis from a point.
//  2.  Determine that if there are an odd number of intersections
//      between this half-line and the polygon, it is inside, and if 
//      there are an even number, it is outside.
bool SketchTool::insideSketch(double x, double y) {
  const double EPS = 1e-5; 
  const int sketchPointsSize = sketchPoints.size();

  int crossCount = 0;
  for (int i = 0; i < sketchPointsSize; i++) {
    glm::dvec3 mappedU = screen.mapCoordinates(sketchPoints[i]);
    glm::dvec3 mappedV = screen.mapCoordinates(sketchPoints[(i + 1) % sketchPointsSize]);

    glm::dvec2 u = glm::dvec2(mappedU.x, mappedU.y);
    glm::dvec2 v = glm::dvec2(mappedV.x, mappedV.y);

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

// Return the pointer to member variables.
PointCloud* SketchTool::getPointCloud() {
  return pointCloud;
}
double SketchTool::getAverageDepth() {
  return averageDepth;
}
glm::dvec3 SketchTool::getCameraOrig() {
  return cameraOrig;
}
glm::dvec3 SketchTool::getCameraDir() {
  return cameraDir;
}
Plane* SketchTool::getScreen() {
  return &screen;
}
std::vector<glm::dvec3>* SketchTool::getSketchPoints() {
  return &sketchPoints;
}
std::vector<int>* SketchTool::getBasisPointsIndex() {
  return &basisPointsIndex;
}

void SketchTool::extendSketchedArea() {
  // Calculate gravity point
  glm::dvec3 gravityPoint = glm::dvec3(0.0, 0.0, 0.0);
  for (glm::dvec3 p: sketchPoints) {
    gravityPoint += p;
  }
  gravityPoint /= static_cast<double>(sketchPoints.size());

  // Calculate averageDistance casted onto screen
  double objectDist = glm::length(cameraOrig);
  double castedAverageDist = pointCloud->getAverageDistance()*screenDist/objectDist;

  // Extend sketched area
  for (int i = 0; i < sketchPoints.size(); i++) {
    glm::dvec3 p = sketchPoints[i];

    // Double the average distance, because the user 
    // should sketch with reference to the boundary of pseudo surface.
    sketchPoints[i] = p + 2.0*castedAverageDist*glm::normalize(p - gravityPoint);
  }
}