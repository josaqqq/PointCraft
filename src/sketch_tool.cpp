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
  double nearClip = polyscope::view::nearClipRatio*polyscope::state::lengthScale * 10.0;
  glm::dvec3 cameraOrig = polyscope::view::getCameraWorldPosition();
  glm::dvec3 cameraDir = polyscope::view::screenCoordsToWorldRay(
    glm::vec2(polyscope::view::windowWidth/2, polyscope::view::windowHeight/2)
  );
  glm::dvec3 screenOrig = cameraOrig + nearClip*cameraDir;

  // Plane on nearClip
  screen = Plane(screenOrig, cameraDir);
}

// Reset all member variables.
void SketchTool::resetSketch() {
  *currentMode = 0;
  averageDepth = 0.0;

  sketchPoints.clear();
  basisPointsIndex.clear();
  discretizedPoints.clear();
}

/*
  Viewer functions
*/

// Register/Remove point cloud with name.
// Be aware that the point cloud with 
// the same name is overwritten.
void SketchTool::registerDiscretizedPointsAsPontCloud(std::string name) {
  // Show discretizedPoints

  std::vector<std::vector<double>> points;
  for (int i = 0; i < discretizedPoints.size(); i++) {
    points.push_back({
      discretizedPoints[i].x,
      discretizedPoints[i].y,
      discretizedPoints[i].z,
    });
  }

  polyscope::PointCloud* patchCloud = polyscope::registerPointCloud(name, points);
  patchCloud->setPointColor(DiscretizedPointColor);
  patchCloud->setPointRadius(DiscretizedPointRadius);
}
void SketchTool::registerBasisPointsAsPointCloud(std::string name) {
  // Show basisPoints

  std::vector<std::vector<double>> points;
  std::vector<std::vector<double>> normals;
  for (int i = 0; i < basisPointsIndex.size(); i++) {
    int idx = basisPointsIndex[i];

    points.push_back({
      pointCloud->meshV(idx, 0),
      pointCloud->meshV(idx, 1),
      pointCloud->meshV(idx, 2)
    });
    normals.push_back({
      pointCloud->meshN(idx, 0),
      pointCloud->meshN(idx, 1),
      pointCloud->meshN(idx, 2)
    });
  }

  polyscope::PointCloud* patchCloud = polyscope::registerPointCloud(name, points);
  patchCloud->setPointColor(BasisPointColor);
  patchCloud->setPointRadius(BasisPointRadius);

  polyscope::PointCloudVectorQuantity *patchVectorQuantity = patchCloud->addVectorQuantity(NormalName, normals);
  patchVectorQuantity->setVectorColor(BasisPointColor);
  patchVectorQuantity->setVectorLengthScale(NormalLength);
  patchVectorQuantity->setVectorRadius(NormalRadius);
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
void SketchTool::registerSketchAsCurveNetworkLoop(std::string name) {
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

// Add the specified point to sketchPoints
void SketchTool::addSketchPoint(glm::dvec3 p) {
  sketchPoints.push_back(p);
}

// Discretize the basis and update discretizedPoints.
void SketchTool::discretizeSketchPoints() {
  const int polygonSize = sketchPoints.size();

  // Comput the search basis.
  const double INF = 100000.0;
  double min_x = INF, max_x = -INF;
  double min_y = INF, max_y = -INF;
  for (int i = 0; i < polygonSize; i++) {
    glm::dvec3 mappedSketchPoint = screen.mapCoordinates(sketchPoints[i]);

    min_x = std::min(min_x, mappedSketchPoint.x);
    max_x = std::max(max_x, mappedSketchPoint.x);
    min_y = std::min(min_y, mappedSketchPoint.y);
    max_y = std::max(max_y, mappedSketchPoint.y);
  }

  double gridArea = (max_x - min_x)*(max_y - min_y) / PatchSize;
  double gridEdge = sqrt(gridArea);

  for (double x = min_x; x < max_x; x += gridEdge) {
    for (double y = min_y; y < max_y; y += gridEdge) {
      if (!insidePolygon(x, y, polygonSize)) continue;
      discretizedPoints.push_back(
        screen.unmapCoordinates(glm::dvec3(x, y, 0.0))
      );
    }
  }
}

// Select the basis points the user selected
// and update sketchPoints.
void SketchTool::findBasisPoints() {
  std::unordered_set<int> selectedPointsIndexSet;

  // Cast a ray from discretized points to point cloud.
  glm::dvec3 cameraOrig = polyscope::view::getCameraWorldPosition();
  for (int i = 0; i < discretizedPoints.size(); i++) {
    Ray ray(cameraOrig, discretizedPoints[i]);
    std::vector<Hit> hitsInfo = ray.searchNeighborPoints(pointCloud->averageDistance, pointCloud);
    for (Hit hitInfo: hitsInfo) {
      if (!hitInfo.hit) continue;
      selectedPointsIndexSet.insert(hitInfo.index);
    }
  }

  std::vector<int> selectedPointsIndex;
  for (int idx: selectedPointsIndexSet) selectedPointsIndex.push_back(idx);

  // Depth detection with DBSCAN
  Clustering clustering(&selectedPointsIndex, pointCloud, &screen);
  basisPointsIndex = clustering.executeDBSCAN(DBSCAN_SearchRange*pointCloud->averageDistance, DBSCAN_MinPoints);
}

// Return the pointer to member variables.
PointCloud* SketchTool::getPointCloud() {
  return pointCloud;
}
double SketchTool::getAverageDepth() {
  return averageDepth;
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
std::vector<glm::dvec3>* SketchTool::getDiscretizedPoints() {
  return &discretizedPoints;
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
    glm::dvec3 mappedU = screen.mapCoordinates(sketchPoints[i]);
    glm::dvec3 mappedV = screen.mapCoordinates(sketchPoints[(i + 1) % polygonSiz]);

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