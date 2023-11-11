#include "polyscope/polyscope.h"

#include "polyscope/pick.h"
#include "polyscope/point_cloud.h"
#include "polyscope/view.h"

#include <string>

#include "ray.hpp"
#include "modes.hpp"

// Patch info
int PatchNum = 0;
std::vector<std::vector<double>> Patch;
std::vector<std::vector<double>> PatchNormal;

const glm::vec3 PatchColor = { 0.000, 1.000, 0.000 };
const double PatchRadius = 0.003;

const std::string PatchNormalName = "normal vector";
const glm::vec3 PatchNormalColor = { 1.000, 0.000, 0.000 };
const double PatchNormalLength = 0.015;
const double PatchNormalRadius = 0.001;
const bool PatchNormalEnabled = true;

// Register Patch with patchName.
// Be aware that the point cloud with 
// the same name is overwritten.
void registerPatch(std::string patchName) {
  // for (size_t i = 0; i < Patch.size(); i++) {
  //   for (size_t j = 0; j < Patch[i].size(); j++) {
  //     std::cout << Patch[i][j] << " ";
  //   }
  //   std::cout << std::endl;
  // }

  polyscope::PointCloud* patch = polyscope::registerPointCloud(patchName + std::to_string(PatchNum), Patch);
  patch->setPointColor(PatchColor);
  patch->setPointRadius(PatchRadius);

  polyscope::PointCloudVectorQuantity *vectorQuantity = patch->addVectorQuantity(PatchNormalName, PatchNormal);
  vectorQuantity->setVectorColor(PatchNormalColor);
  vectorQuantity->setVectorLengthScale(PatchNormalLength);
  vectorQuantity->setVectorRadius(PatchNormalRadius);
  vectorQuantity->setEnabled(PatchNormalEnabled);
}

void resetMode(Mode &currentMode) {
  // TODO: Guard for dragging on windows. checkbox is sufficient?
  PatchNum++;
  Patch.clear();
  PatchNormal.clear();
  currentMode = MODE_NONE;
}

// Pick a point by the mouse position, 
// then add the point to Patch.
void tracePoints(ImGuiIO &io, Mode &currentMode) {
  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    ImVec2 mousePos = ImGui::GetMousePos();
    int xPos = io.DisplayFramebufferScale.x * mousePos.x;
    int yPos = io.DisplayFramebufferScale.y * mousePos.y;

    // Pick a point
    std::pair<polyscope::Structure*, size_t> pickResult = polyscope::pick::evaluatePickQuery(xPos, yPos);
    if (pickResult.first == nullptr) return;

    // TODO: 
    // If SurfaceMesh is picked, this program
    // crashes here. We need to only search
    // for vertices.
    polyscope::PointCloud* pointCloud = polyscope::getPointCloud(pickResult.first->name);
    if (pointCloud == nullptr) return;

    // Get point position and push it to Patch
    glm::vec3 pointPos = pointCloud->getPointPosition(pickResult.second);
    // TODO: Guard for duplicated points.
    Patch.push_back({
      pointPos.x, 
      pointPos.y, 
      pointPos.z
    });
    PatchNormal.push_back({
      0.0,
      0.0,
      0.0,
    });

    // Register Patch as point cloud.
    registerPatch("trace: ");
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

    // TODO: Guard for duplicated points.
    Patch.push_back({
      hitInfo.pos.x, 
      hitInfo.pos.y, 
      hitInfo.pos.z
    });
    PatchNormal.push_back({
      hitInfo.normal.x,
      hitInfo.normal.y,
      hitInfo.normal.z
    });

    // Register Patch as point cloud.
    registerPatch("cast: ");
  }

  if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && Patch.size() > 0) {
    resetMode(currentMode);
  }
}
