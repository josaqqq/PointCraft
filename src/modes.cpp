#include "polyscope/polyscope.h"

#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"

#include "modes.hpp"
#include "surface.hpp"
#include "constants.hpp"

void ModeSelector::enableModeSelection(ImGuiIO &io) {
  // Tool Selection
  ImGui::Text("Tool Selection:");
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Reset Tool")) {
    *currentMode = MODE_NONE;
  }
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Interpolation Tool")) {
    *currentMode = MODE_INTERPOLATION;
    interpolationTool->initSketch();
  }

  switch (*currentMode) {
    case MODE_NONE:
      ImGui::Text("   Selected Tool: None");
      polyscope::view::moveScale = 1.0;
      break;
    case MODE_INTERPOLATION:
      ImGui::Text("   Selected Tool: Interpolation Tool");
      polyscope::view::moveScale = 0.0;
      if (interpolationTool->drawSketch()) *currentSurfaceMode = SURFACE_MODE_PSEUDO;
      break;
  }

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

  // Surface Reconstruction
  ImGui::Text("\nSurface Reconstruction");
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Reconstruct Poisson Surface")) {
    poissonReconstruct(
      PoissonName,
      pointCloud->getAverageDistance(),
      pointCloud->Vertices,
      pointCloud->Normals
    );
    *currentSurfaceMode = SURFACE_MODE_POISSON;
  }
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Reconstruct Greedy Surface")) {
    greedyProjection(
      GreedyProjName,
      pointCloud->Vertices,
      pointCloud->Normals
    );
    *currentSurfaceMode = SURFACE_MODE_GREEDY;
  }

  // Surface Visualization
  ImGui::Text("\nSurface Visualization:");
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("None", currentSurfaceMode, SURFACE_MODE_NONE);
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("Pseudo Surface", currentSurfaceMode, SURFACE_MODE_PSEUDO);
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("Poisson Surface", currentSurfaceMode, SURFACE_MODE_POISSON);
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("Greedy Surface", currentSurfaceMode, SURFACE_MODE_GREEDY);

  polyscope::SurfaceMesh *pseudoSurface = polyscope::getSurfaceMesh(PseudoSurfaceName);
  polyscope::SurfaceMesh *poissonSurface = polyscope::getSurfaceMesh(PoissonName);
  polyscope::SurfaceMesh *greedySurface = polyscope::getSurfaceMesh(GreedyProjName);
  switch (*currentSurfaceMode) {
    case SURFACE_MODE_NONE:
      pseudoSurface->setEnabled(false);
      poissonSurface->setEnabled(false);
      greedySurface->setEnabled(false);
      break;
    case SURFACE_MODE_PSEUDO:
      pseudoSurface->setEnabled(true);
      poissonSurface->setEnabled(false);
      greedySurface->setEnabled(false);
      break;
    case SURFACE_MODE_POISSON:
      pseudoSurface->setEnabled(false);
      poissonSurface->setEnabled(true);
      greedySurface->setEnabled(false);
      break;
    case SURFACE_MODE_GREEDY:
      pseudoSurface->setEnabled(false);
      poissonSurface->setEnabled(false);
      greedySurface->setEnabled(true);
      break;
  }
}