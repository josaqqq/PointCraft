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
std::unordered_set<glm::vec3>     PointSet;
std::vector<Hit>                  HitPoints;

// When left click is released, reset the status and mode.
void resetMode(int &currentMode) {
  // TODO: Guard for dragging on windows. checkbox is sufficient?
  PatchNum++;
  averageDepth = 0.0;
  PointSet.clear();
  HitPoints.clear();
  currentMode = MODE_NONE;
}

// Register/Remove Patch as point cloud with patchName.
// Be aware that the point cloud with 
// the same name is overwritten.
extern Eigen::MatrixXd meshV;
extern Eigen::MatrixXd meshN;
void registerPatchAsPointCloud(std::string patchName, bool replaceMeshV = false) {
  patchName = patchName + std::to_string(PatchNum);

  Eigen::MatrixXd meshNewV(HitPoints.size(), 3);
  Eigen::MatrixXd meshNewN(HitPoints.size(), 3);
  for (int i = 0; i < HitPoints.size(); i++) {
    meshNewV(i, 0) = HitPoints[i].pos.x;
    meshNewV(i, 1) = HitPoints[i].pos.y;
    meshNewV(i, 2) = HitPoints[i].pos.z;

    meshNewN(i, 0) = HitPoints[i].normal.x;
    meshNewN(i, 1) = HitPoints[i].normal.y;
    meshNewN(i, 2) = HitPoints[i].normal.z;
  }

  if (replaceMeshV) {
    patchName = PointName;

    meshNewV.conservativeResize(meshNewV.rows() + meshV.rows(), Eigen::NoChange);
    meshNewN.conservativeResize(meshNewN.rows() + meshN.rows(), Eigen::NoChange);

    for (int i = 0; i < meshV.rows(); i++) {
      meshNewV.row(HitPoints.size() + i) = meshV.row(i);
      meshNewN.row(HitPoints.size() + i) = meshN.row(i);
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
  std::vector<std::vector<double>> Patch;
  for (int i = 0; i < HitPoints.size(); i++) {
    Patch.push_back({
      HitPoints[i].pos.x,
      HitPoints[i].pos.y,
      HitPoints[i].pos.z
    });
  }
  if (HitPoints.size() == 1) {
    Patch.push_back({
      HitPoints[0].pos.x,
      HitPoints[0].pos.y,
      HitPoints[0].pos.z
    });
  }

  patchName = patchName + std::to_string(PatchNum);

  polyscope::CurveNetwork* sketch = polyscope::registerCurveNetworkLine(patchName, Patch);
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
  std::vector<std::vector<double>> Patch;
  for (int i = 0; i < HitPoints.size(); i++) {
    Patch.push_back({
      HitPoints[i].pos.x,
      HitPoints[i].pos.y,
      HitPoints[i].pos.z
    });
  }

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
bool addPointToPatch(Hit hitInfo) {
  if (PointSet.count(hitInfo.pos)) return false;
  if (HitPoints.size() > 0) {
    if (std::abs(hitInfo.depth - averageDepth) >= averageDistance * depthInterval) return false;
  }
  averageDepth = (averageDepth*HitPoints.size() + hitInfo.depth)/(HitPoints.size() + 1);
  PointSet.insert(hitInfo.pos);
  HitPoints.push_back(hitInfo);

  return true;
}

// Check the inside/outside of the polygon.
//  1.  Draw a half-line parallel to the x-axis from a point.
//  2.  Determine that if there are an odd number of intersections
//      between this half-line and the polygon, it is inside, and if 
//      there are an even number, it is outside.
bool inside_polygon(double x, double y, const int polygonSiz) {
  const double EPS = 1e-5; 

  int crossCount = 0;
  for (int i = 0; i < polygonSiz; i++) {
    glm::dvec2 u = glm::dvec2(HitPoints[i].screenCoord.x, HitPoints[i].screenCoord.y);
    glm::dvec2 v = glm::dvec2(HitPoints[(i + 1) % polygonSiz].screenCoord.x, HitPoints[(i + 1) % polygonSiz].screenCoord.y);

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
// -> this function cast points to the specified sphere.
void fillSketchedArea(glm::vec3 center, double radius) {
  const int polygonSiz = HitPoints.size();

  // Compute the search boundary.
  const float INF = 100000.0;
  float min_x = INF, max_x = -INF;
  float min_y = INF, max_y = -INF;
  for (int i = 0; i < polygonSiz; i++) {
    min_x = std::min(min_x, HitPoints[i].screenCoord.x);
    max_x = std::max(max_x, HitPoints[i].screenCoord.x);
    min_y = std::min(min_y, HitPoints[i].screenCoord.y);
    max_y = std::max(max_y, HitPoints[i].screenCoord.y);
  }

  // Adjust grid size to keep the number
  // of points to be added constant.
  float gridNum = (glm::ceil(max_x) - glm::floor(min_x)) * (glm::ceil(max_y) - glm::floor(min_y));
  int gridSize = glm::sqrt(gridNum / PatchSize);

  // Inside/Outside Check for each filled piont.
  for (int i = glm::floor(min_x); i < glm::ceil(max_x); i += gridSize) {
    for (int j = glm::floor(min_y); j < glm::ceil(max_y); j += gridSize) {
      if (!inside_polygon(i, j, polygonSiz)) continue;

      // Cast a ray
      Ray ray(i, j);
      Hit hitInfo = ray.checkSphere(center, radius);
      if (!hitInfo.hit) continue;
      addPointToPatch(hitInfo);
    } 
  }
}

// Compute the search boundary, and then fill
// the inside are of the polygon with points.
// -> this function cast points to the approximate surface.
void fillSketchedArea(double depth) {
  const int polygonSiz = HitPoints.size();

  // Compute the search boundary.
  const float INF = 100000.0;
  float min_x = INF, max_x = -INF;
  float min_y = INF, max_y = -INF;
  for (int i = 0; i < polygonSiz; i++) {
    min_x = std::min(min_x, HitPoints[i].screenCoord.x);
    max_x = std::max(max_x, HitPoints[i].screenCoord.x);
    min_y = std::min(min_y, HitPoints[i].screenCoord.y);
    max_y = std::max(max_y, HitPoints[i].screenCoord.y);
  }

  // Adjust grid size to keep the number
  // of points to be added constant.
  float gridNum = (glm::ceil(max_x) - glm::floor(min_x)) * (glm::ceil(max_y) - glm::floor(min_y));
  int gridSize = glm::sqrt(gridNum / PatchSize);

  // Inside/Outside Check for each filled piont.
  for (int i = glm::floor(min_x); i < glm::ceil(max_x); i += gridSize) {
    for (int j = glm::floor(min_y); j < glm::ceil(max_y); j += gridSize) {
      if (!inside_polygon(i, j, polygonSiz)) continue;

      // Cast a ray
      Ray ray(i, j);
      Hit hitInfo = ray.castPointToPlane(depth);
      if (!hitInfo.hit) continue;
      addPointToPatch(hitInfo);
    } 
  }
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
    addPointToPatch(hitInfo);

    // Register Patch as curve network (LINE)
    registerPatchAsCurveNetworkLine(TracePrefix);
  }

  if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && HitPoints.size() > 0) {
    // Fill the sketched area.
    fillSketchedArea(averageDepth);

    // Register patch as point cloud.
    registerPatchAsPointCloud(CastPrefix, true);
    removePatchAsCurveNetworkLine(TracePrefix);

    // Reconstruct Surface
    poissonReconstruct(meshV, meshN);
    greedyProjection(meshV, meshN);

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
    addPointToPatch(hitInfo);

    // Register Patch as point cloud.
    registerPatchAsPointCloud(CastPrefix);
  }

  if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && HitPoints.size() > 0) {
    resetMode(currentMode);
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
    addPointToPatch(hitInfo);

    // Register Patch as curve network (LINE)
    registerPatchAsCurveNetworkLine(SketchPrefix);
  }

  if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && HitPoints.size() > 0) {

    // Fill the sketched area.
    fillSketchedArea(center, radius);

    // Register the point cloud. 
    registerPatchAsPointCloud(CastPrefix, true);
    removePatchAsCurveNetworkLine(SketchPrefix);

    // Reconstruct Surface
    poissonReconstruct(meshV, meshN);
    greedyProjection(meshV, meshN);

    resetMode(currentMode);
  }
}
