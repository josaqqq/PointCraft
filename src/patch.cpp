#include "polyscope/polyscope.h"

#include "polyscope/pick.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"
#include "polyscope/view.h"

#include <glm/gtx/hash.hpp>

#include <Eigen/Dense>

#include <string>
#include <unordered_set>

#include "ray.hpp"
#include "modes.hpp"
#include "surface.hpp"
#include "constants.hpp"

// Patch info
int PatchNum = 0;
double averageDepth = 0.0;
std::unordered_set<glm::vec3>     PatchSet;
std::vector<glm::vec2>            ScreenCoords;
std::vector<std::vector<double>>  Patch;
std::vector<std::vector<double>>  PatchNormal;

// When left click is released, reset the status and mode.
void resetMode(int &currentMode) {
  // TODO: Guard for dragging on windows. checkbox is sufficient?
  PatchNum++;
  averageDepth = 0.0;
  PatchSet.clear();
  ScreenCoords.clear();
  Patch.clear();
  PatchNormal.clear();
  currentMode = MODE_NONE;
}

// Register/Remove Patch as point cloud with patchName.
// Be aware that the point cloud with 
// the same name is overwritten.
extern Eigen::MatrixXd meshV;
extern Eigen::MatrixXd meshN;
void registerPatchAsPointCloud(std::string patchName, bool replaceMeshV = false) {
  patchName = patchName + std::to_string(PatchNum);

  Eigen::MatrixXd meshNewV(Patch.size(), 3);
  Eigen::MatrixXd meshNewN(Patch.size(), 3);
  for (int i = 0; i < Patch.size(); i++) {
    meshNewV(i, 0) = Patch[i][0];
    meshNewV(i, 1) = Patch[i][1];
    meshNewV(i, 2) = Patch[i][2];

    meshNewN(i, 0) = PatchNormal[i][0];
    meshNewN(i, 1) = PatchNormal[i][1];
    meshNewN(i, 2) = PatchNormal[i][2];
  }

  if (replaceMeshV) {
    patchName = PointName;

    meshNewV.conservativeResize(meshNewV.rows() + meshV.rows(), Eigen::NoChange);
    meshNewN.conservativeResize(meshNewN.rows() + meshN.rows(), Eigen::NoChange);

    for (int i = 0; i < meshV.rows(); i++) {
      meshNewV.row(Patch.size() + i) = meshV.row(i);
      meshNewN.row(Patch.size() + i) = meshN.row(i);
    }

    meshV = meshNewV;
    meshN = meshNewN;
  }

  polyscope::PointCloud* patch = polyscope::registerPointCloud(patchName, meshNewV);
  patch->setPointColor(PointColor);
  patch->setPointRadius(PointRadius);

  polyscope::PointCloudVectorQuantity *vectorQuantity = patch->addVectorQuantity(NormalName, meshNewN);
  vectorQuantity->setVectorColor(NormalColor);
  vectorQuantity->setVectorLengthScale(NormalLength);
  vectorQuantity->setVectorRadius(NormalRadius);
  vectorQuantity->setEnabled(NormalEnabled);
}
void removePatchAsPointCloud(std::string patchName) {
  patchName = patchName + std::to_string(PatchNum);
  polyscope::removePointCloud(patchName, false);
}

// Register/Remove Patch as curve network line with patchName.
// Be aware that the curve network with 
// the same name is overwritten
void registerPatchAsCurveNetworkLine(std::string patchName) {
  std::vector<std::vector<double>>  PatchDummy = Patch;
  if (PatchDummy.size() == 1) PatchDummy.push_back(PatchDummy[0]);

  patchName = patchName + std::to_string(PatchNum);

  polyscope::CurveNetwork* sketch = polyscope::registerCurveNetworkLine(patchName, PatchDummy);
  sketch->setColor(CurveNetworkColor);
  sketch->setRadius(CurveNetworkRadius);
}
void removePatchAsCurveNetworkLine(std::string patchName) {
  patchName = patchName + std::to_string(PatchNum);
  polyscope::removeCurveNetwork(patchName, false);
}

// Register/Remove Patch as curve network loop with patchName.
// Be aware that the curve network with 
// the same name is overwritten
void registerPatchAsCurveNetworkLoop(std::string patchName) {
  patchName = patchName + std::to_string(PatchNum);

  polyscope::CurveNetwork* sketch = polyscope::registerCurveNetworkLoop(patchName, Patch);
  sketch->setColor(CurveNetworkColor);
  sketch->setRadius(CurveNetworkRadius);
}
void removePatchAsCurveNetworkLoop(std::string patchName) {
  patchName = patchName + std::to_string(PatchNum);
  polyscope::removeCurveNetwork(patchName, false);
}

// If the point is already added, then skip it.
// If the depth of the added points is out of the range, then skip it.
extern double averageDistance;
bool addPointToPatch(double depth, glm::vec3 pointPos, glm::vec3 normalVec) {
  if (PatchSet.count(pointPos)) return false;
  if (Patch.size() > 0) {
    if (std::abs(depth - averageDepth) >= averageDistance * depthInterval) return false;
  }

  PatchSet.insert(pointPos);
  
  averageDepth = (averageDepth*Patch.size() + depth)/(Patch.size() + 1);

  Patch.push_back({
    pointPos.x,
    pointPos.y,
    pointPos.z,
  });

  PatchNormal.push_back({
    normalVec.x,
    normalVec.y,
    normalVec.z
  });

  return true;
}

// Pick a point by the mouse position, 
// then add the point to Patch.
void tracePoints(ImGuiIO &io, int &currentMode) {
  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    ImVec2 mousePos = ImGui::GetMousePos();
    int xPos = io.DisplayFramebufferScale.x * mousePos.x;
    int yPos = io.DisplayFramebufferScale.y * mousePos.y;

    // Cast a ray
    Ray ray(xPos, yPos);
    Hit hitInfo = ray.searchNeighborPoints(CurveNetworkRadius);
    if (!hitInfo.hit) return;
    addPointToPatch(hitInfo.t, hitInfo.pos, hitInfo.normal);

    // Register Patch as point cloud.
    registerPatchAsCurveNetworkLine("trace: ");
  }

  if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && Patch.size() > 0) {
    resetMode(currentMode);
  }
}

// Cast a ray from the mouse position,
// then add the intersection point with sphere
// to Patch.
void castPointToSphere(ImGuiIO &io, int &currentMode, glm::vec3 center, double radius) {
  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    ImVec2 mousePos = ImGui::GetMousePos();
    int xPos = io.DisplayFramebufferScale.x * mousePos.x;
    int yPos = io.DisplayFramebufferScale.y * mousePos.y;

    // Cast a ray
    Ray ray(xPos, yPos);
    Hit hitInfo = ray.checkSphere(center, radius);
    if (!hitInfo.hit) return;
    addPointToPatch(hitInfo.t, hitInfo.pos, hitInfo.normal);

    // Register Patch as point cloud.
    registerPatchAsPointCloud("cast: ");
  }

  if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && Patch.size() > 0) {
    resetMode(currentMode);
  }
}

// Check the inside/outside of the polygon.
//  1.  Draw a half-line parallel to the x-axis from a point.
//  2.  Determine that if there are an odd number of intersections
//      between this half-line and the polygon, it is inside, and if 
//      there are an even number, it is outside.
bool inside_polygon(double x, double y) {
  const double EPS = 1e-5; 
  const int polygonSiz = ScreenCoords.size();

  int crossCount = 0;
  for (int i = 0; i < polygonSiz; i++) {
    glm::dvec2 u = glm::dvec2(ScreenCoords[i][0], ScreenCoords[i][1]);
    glm::dvec2 v = glm::dvec2(ScreenCoords[(i + 1) % polygonSiz][0], ScreenCoords[(i + 1) % polygonSiz][1]);

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

// Compute the search boundary, and then fill
// the inside are of the polygon with points.
void fillSketchedArea(glm::vec3 center, double radius) {
  // Compute the search boundary.
  const float INF = 100000.0;
  float min_x = INF, max_x = -INF;
  float min_y = INF, max_y = -INF;
  for (int i = 0; i < ScreenCoords.size(); i++) {
    min_x = std::min(min_x, ScreenCoords[i].x);
    max_x = std::max(max_x, ScreenCoords[i].x);
    min_y = std::min(min_y, ScreenCoords[i].y);
    max_y = std::max(max_y, ScreenCoords[i].y);
  }

  // Adjust grid size to keep the number
  // of points to be added constant.
  float gridNum = (glm::ceil(max_x) - glm::floor(min_x)) * (glm::ceil(max_y) - glm::floor(min_y));
  int gridSize = glm::sqrt(gridNum / PatchSize);

  // Inside/Outside Check for each filled piont.
  for (int i = glm::floor(min_x); i < glm::ceil(max_x); i += gridSize) {
    for (int j = glm::floor(min_y); j < glm::ceil(max_y); j += gridSize) {
      if (!inside_polygon(i, j)) continue;

      // Cast a ray
      Ray ray(i, j);
      Hit hitInfo = ray.checkSphere(center, radius);
      if (!hitInfo.hit) continue;
    addPointToPatch(hitInfo.t, hitInfo.pos, hitInfo.normal);
    } 
  }
}

// While dragging the mouse, display the CurveNetwork.
// When the mouse released, fill the sketched area and display them as PointCloud.
void createPatchToPointCloud(ImGuiIO &io, int &currentMode, glm::vec3 center, double radius) {
  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    ImVec2 mousePos = ImGui::GetMousePos();
    int xPos = io.DisplayFramebufferScale.x * mousePos.x;
    int yPos = io.DisplayFramebufferScale.y * mousePos.y;

    // Cast a ray
    Ray ray(xPos, yPos);
    Hit hitInfo = ray.checkSphere(center, radius);
    if (!hitInfo.hit) return;
    if (addPointToPatch(hitInfo.t, hitInfo.pos, hitInfo.normal)) {
      ScreenCoords.push_back(hitInfo.screenCoord);
    }

    // Register Patch as point cloud.
    registerPatchAsCurveNetworkLine("sketch ");
  }

  if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && Patch.size() > 0) {
    // Fill the sketched area.
    fillSketchedArea(center, radius);

    // Register the point cloud, and then remove curve network 
    registerPatchAsPointCloud("cast: ", true);
    removePatchAsCurveNetworkLine("sketch ");

    // Reconstruct Surface
    poissonReconstruct(meshV, meshN);
    greedyProjection(meshV, meshN);

    resetMode(currentMode);
  }
}
