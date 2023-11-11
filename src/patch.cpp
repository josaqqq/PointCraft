#include "polyscope/polyscope.h"

#include "polyscope/pick.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"
#include "polyscope/view.h"

#include <glm/gtx/hash.hpp>

#include <string>
#include <unordered_set>

#include "ray.hpp"
#include "modes.hpp"

// Patch info
int PatchNum = 0;
std::unordered_set<glm::vec3>     PatchSet;
std::vector<std::vector<double>>  Patch;
std::vector<std::vector<double>>  PatchNormal;

const glm::vec3 PatchColor  = { 0.000, 1.000, 0.000 };
const double PatchRadius    = 0.003;

const std::string PatchNormalName = "normal vector";
const glm::vec3 PatchNormalColor  = { 1.000, 0.000, 0.000 };
const double PatchNormalLength    = 0.015;
const double PatchNormalRadius    = 0.001;
const bool PatchNormalEnabled     = true;

// Register Patch as point cloud with patchName.
// Be aware that the point cloud with 
// the same name is overwritten.
void registerPatchAsPointCloud(std::string patchName) {
  polyscope::PointCloud* patch = polyscope::registerPointCloud(patchName + std::to_string(PatchNum), Patch);
  patch->setPointColor(PatchColor);
  patch->setPointRadius(PatchRadius);

  polyscope::PointCloudVectorQuantity *vectorQuantity = patch->addVectorQuantity(PatchNormalName, PatchNormal);
  vectorQuantity->setVectorColor(PatchNormalColor);
  vectorQuantity->setVectorLengthScale(PatchNormalLength);
  vectorQuantity->setVectorRadius(PatchNormalRadius);
  vectorQuantity->setEnabled(PatchNormalEnabled);
}

// Register Patch as curve network line with patchName.
// Be aware that the curve network with 
// the same name is overwritten
void registerPatchAsCurveNetworkLine(std::string patchName) {
  if (Patch.size() == 1) return;

  polyscope::CurveNetwork* sketch = polyscope::registerCurveNetworkLine(patchName + std::to_string(PatchNum), Patch);
  sketch->setColor(PatchColor);
  sketch->setRadius(PatchRadius);
}

// Register Patch as curve network loop with patchName.
// Be aware that the curve network with 
// the same name is overwritten
void registerPatchAsCurveNetworkLoop(std::string patchName) {
  polyscope::CurveNetwork* sketch = polyscope::registerCurveNetworkLoop(patchName + std::to_string(PatchNum), Patch);
  sketch->setColor(PatchColor);
  sketch->setRadius(PatchRadius);
}

// When left click is released, reset the status and mode.
void resetMode(Mode &currentMode) {
  // TODO: Guard for dragging on windows. checkbox is sufficient?
  PatchNum++;
  Patch.clear();
  PatchSet.clear();
  PatchNormal.clear();
  currentMode = MODE_NONE;
}

// If the point is not added yet, add the information of the point.
void addPointToPatch(glm::vec3 pointPos, glm::vec3 normalVec) {
  if (PatchSet.count(pointPos)) return;

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
}

// Pick a point by the mouse position, 
// then add the point to Patch.
void tracePoints(ImGuiIO &io, Mode &currentMode) {
  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    ImVec2 mousePos = ImGui::GetMousePos();
    int xPos = io.DisplayFramebufferScale.x * mousePos.x;
    int yPos = io.DisplayFramebufferScale.y * mousePos.y;

    // Pick a point
    
    // Using default function: evaluatePickQuery.
      // std::pair<polyscope::Structure*, size_t> pickResult = polyscope::pick::evaluatePickQuery(xPos, yPos);
      // if (pickResult.first == nullptr) return;
      // if (pickResult.first->typeName() != "Point Cloud") return;  // If the selected structure is not PointCloud, then return.

      // polyscope::PointCloud* pointCloud = polyscope::getPointCloud(pickResult.first->name);
      // if (pointCloud == nullptr) return;

      // // Get point position and push it to Patch
      // glm::vec3 pointPos = pointCloud->getPointPosition(pickResult.second);
      // addPointToPatch(pointPos, glm::vec3(0.0, 0.0, 0.0));

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

void createCurveNetwork(ImGuiIO &io, Mode &currentMode, glm::vec3 center, double radius) {
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
    registerPatchAsCurveNetworkLoop("sketch ");
  }

  if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && Patch.size() > 0) {
    resetMode(currentMode);
  }
}
