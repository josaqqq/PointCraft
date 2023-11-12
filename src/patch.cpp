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

// Patch info
int PatchNum = 0;
std::unordered_set<glm::vec3>     PatchSet;
std::vector<glm::vec2>            ScreenCoords;
std::vector<std::vector<double>>  Patch;
std::vector<std::vector<double>>  PatchNormal;

const int ScreenGrid = 15;

const glm::vec3 PatchColor = { 1.000, 1.000, 1.000 };
const double PatchRadius = 0.002;

const std::string PatchNormalName = "normal vector";
const glm::vec3 PatchNormalColor =  {1.000, 0.000, 0.000 };
const double PatchNormalLength = 0.015;
const double PatchNormalRadius = 0.001;
const bool PatchNormalEnabled = true;

// TODO: Change color and size for added points.

// const glm::vec3 PatchColor  = { 0.000, 1.000, 0.000 };
// const double PatchRadius    = 0.003;

// const std::string PatchNormalName = "normal vector";
// const glm::vec3 PatchNormalColor  = { 1.000, 0.000, 0.000 };
// const double PatchNormalLength    = 0.015;
// const double PatchNormalRadius    = 0.001;
// const bool PatchNormalEnabled     = true;

// When left click is released, reset the status and mode.
void resetMode(Mode &currentMode) {
  // TODO: Guard for dragging on windows. checkbox is sufficient?
  PatchNum++;
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
extern std::string PointName;

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
  patch->setPointColor(PatchColor);
  patch->setPointRadius(PatchRadius);

  polyscope::PointCloudVectorQuantity *vectorQuantity = patch->addVectorQuantity(PatchNormalName, meshNewN);
  vectorQuantity->setVectorColor(PatchNormalColor);
  vectorQuantity->setVectorLengthScale(PatchNormalLength);
  vectorQuantity->setVectorRadius(PatchNormalRadius);
  vectorQuantity->setEnabled(PatchNormalEnabled);
}
void removePatchAsPointCloud(std::string patchName) {
  patchName = patchName + std::to_string(PatchNum);
  polyscope::removePointCloud(patchName, false);
}

// Register/Remove Patch as curve network line with patchName.
// Be aware that the curve network with 
// the same name is overwritten
void registerPatchAsCurveNetworkLine(std::string patchName) {
  if (Patch.size() == 1) return;

  patchName = patchName + std::to_string(PatchNum);

  polyscope::CurveNetwork* sketch = polyscope::registerCurveNetworkLine(patchName, Patch);
  sketch->setColor(PatchColor);
  sketch->setRadius(PatchRadius);
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
  sketch->setColor(PatchColor);
  sketch->setRadius(PatchRadius);
}
void removePatchAsCurveNetworkLoop(std::string patchName) {
  patchName = patchName + std::to_string(PatchNum);
  polyscope::removeCurveNetwork(patchName, false);
}

// If the point is not added yet, add the information of the point.
bool addPointToPatch(glm::vec3 pointPos, glm::vec3 normalVec) {
  if (PatchSet.count(pointPos)) return false;

  PatchSet.insert(pointPos);
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
void tracePoints(ImGuiIO &io, Mode &currentMode) {
  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    ImVec2 mousePos = ImGui::GetMousePos();
    int xPos = io.DisplayFramebufferScale.x * mousePos.x;
    int yPos = io.DisplayFramebufferScale.y * mousePos.y;

    // Cast a ray
    Ray ray(xPos, yPos);
    Hit hitInfo = ray.searchNeighborPoints(1.0);
    if (!hitInfo.hit) return;
    addPointToPatch(hitInfo.pos, hitInfo.normal);

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
void castPointToSphere(ImGuiIO &io, Mode &currentMode, glm::vec3 center, double radius) {
  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    ImVec2 mousePos = ImGui::GetMousePos();
    int xPos = io.DisplayFramebufferScale.x * mousePos.x;
    int yPos = io.DisplayFramebufferScale.y * mousePos.y;

    // Cast a ray
    Ray ray(xPos, yPos);
    Hit hitInfo = ray.checkSphere(center, radius);
    if (!hitInfo.hit) return;
    addPointToPatch(hitInfo.pos, hitInfo.normal);

    // Register Patch as point cloud.
    registerPatchAsPointCloud("cast: ");
  }

  if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && Patch.size() > 0) {
    resetMode(currentMode);
  }
}

// Checck the inside/outside of the polygon.
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

  // Inside/Outside Check for each filled piont.
  for (int i = glm::floor(min_x); i < glm::ceil(max_x); i += ScreenGrid) {
    for (int j = glm::floor(min_y); j < glm::ceil(max_y); j += ScreenGrid) {
      if (!inside_polygon(i, j)) continue;

      // Cast a ray
      Ray ray(i, j);
      Hit hitInfo = ray.checkSphere(center, radius);
      if (!hitInfo.hit) continue;
      addPointToPatch(hitInfo.pos, hitInfo.normal);
    } 
  }
}

// While dragging the mouse, display the CurveNetwork.
// When the mouse released, fill the sketched area and display them as PointCloud.
void createPatchToPointCloud(ImGuiIO &io, Mode &currentMode, glm::vec3 center, double radius) {
  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    ImVec2 mousePos = ImGui::GetMousePos();
    int xPos = io.DisplayFramebufferScale.x * mousePos.x;
    int yPos = io.DisplayFramebufferScale.y * mousePos.y;

    // Cast a ray
    Ray ray(xPos, yPos);
    Hit hitInfo = ray.checkSphere(center, radius);
    if (!hitInfo.hit) return;
    if (addPointToPatch(hitInfo.pos, hitInfo.normal)) {
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
