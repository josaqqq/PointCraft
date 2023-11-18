#include "polyscope/polyscope.h"

#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"

#include "modes.hpp"
#include "surface.hpp"
#include "constants.hpp"

void ModeSelector::enableModeSelection(ImGuiIO &io) {
  // Point Cloud Visualization
  // ImGui::Text("Point Cloud Visualization");
  // ImGui::Text("   "); ImGui::SameLine();
  // ImGui::RadioButton("Hide", currentPointCloud, MODE_HIDE);
  // ImGui::Text("   "); ImGui::SameLine();
  // ImGui::RadioButton("Show", currentPointCloud, MODE_SHOW);

  // switch (*currentPointCloud) {
  //   case MODE_HIDE:
  //     pointCloud->setPointCloudEnabled(false);
  //     break;
  //   case MODE_SHOW:
  //     pointCloud->setPointCloudEnabled(true);
  //     break;
  // }

  // Point Cloud Normal Visualization
  // ImGui::Text("Normal Visualization");
  // ImGui::Text("   "); ImGui::SameLine();
  // ImGui::RadioButton("Hide", currentPointCloudNormal, MODE_HIDE);
  // ImGui::Text("   "); ImGui::SameLine();
  // ImGui::RadioButton("Show", currentPointCloudNormal, MODE_SHOW);

  // switch (*currentPointCloudNormal) {
  //   case MODE_HIDE:
  //     pointCloud->setPointCloudNormalEnabled(false);
  //     break;
  //   case MODE_SHOW:
  //     pointCloud->setPointCloudNormalEnabled(true);
  //     break;
  // }

  // Surface Visualization
  ImGui::Text("Surface Visualization");
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("None", currentSurfaceMode, SURFACE_MODE_NONE);
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("Poisson Surface", currentSurfaceMode, SURFACE_MODE_POISSON);
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("Greedy Surface", currentSurfaceMode, SURFACE_MODE_GREEDY);

  polyscope::SurfaceMesh *poissonSurface = polyscope::getSurfaceMesh(PoissonName);
  polyscope::SurfaceMesh *greedySurface = polyscope::getSurfaceMesh(GreedyProjName);
  switch (*currentSurfaceMode) {
    case SURFACE_MODE_NONE:
      poissonSurface->setEnabled(false);
      greedySurface->setEnabled(false);
      break;
    case SURFACE_MODE_POISSON:
      poissonSurface->setEnabled(true);
      greedySurface->setEnabled(false);
      break;
    case SURFACE_MODE_GREEDY:
      poissonSurface->setEnabled(false);
      greedySurface->setEnabled(true);
      break;
  }

  // Tool Selection
  ImGui::Text("Tool Selection:");
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Reset Tool"))          *currentMode = MODE_NONE;
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Interpolation Tool"))  *currentMode = MODE_INTERPOLATION;

  switch (*currentMode) {
    case MODE_NONE:
      ImGui::Text("   Selected Tool: None");
      polyscope::view::moveScale = 1.0;
      break;
    case MODE_INTERPOLATION:
      ImGui::Text("   Selected Tool: Interpolation Tool");
      polyscope::view::moveScale = 0.0;
      interpolationTool->drawSketch();
      break;
  }
}