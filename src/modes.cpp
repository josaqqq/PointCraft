#include "polyscope/polyscope.h"

#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"

#include "modes.hpp"
#include "surface.hpp"
#include "constants.hpp"

void ModeSelector::enableModeSelection(ImGuiIO &io) {
  // Export .obj file
  ImGui::Text("Export file:");
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button(".obj")) pointCloud->exportOBJFile();

  // Undo/Redo
  ImGui::Text("\nUndo/Redo:");
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("<< Undo")) pointCloud->executeUndo();
  ImGui::SameLine();
  if (ImGui::Button("Redo >>")) pointCloud->executeRedo();

  // Tool Selection
  ImGui::Text("\nTool Selection:");
  switch (*currentMode) {
    case MODE_NONE:
      ImGui::Text("   Selected Tool: None");
      polyscope::view::moveScale = 1.0;
      break;
    case MODE_SKETCH_INTERPOLATION:
      ImGui::Text("   Selected Tool: Sketch");
      sketchInterpolationTool->launchToolOperation();
      break;
    case MODE_SPRAY_INTERPOLATION:
      ImGui::Text("   Selected Tool: Spray");
      sprayInterpolationTool->launchToolOperation();
      break;
    case MODE_DELETION:
      ImGui::Text("   Selected Tool: Delete");
      deleteTool->launchToolOperation();
      break;
  }
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Reset")) {
    *currentMode = MODE_NONE;
  }
  ImGui::SameLine();
  if (sketchInterpolationTool != nullptr) {
    if (ImGui::Button("Sketch")) {
      *currentMode = MODE_SKETCH_INTERPOLATION;
      sketchInterpolationTool->initSketch();
    }
    ImGui::SameLine();
  }
  if (sprayInterpolationTool != nullptr) {
    if (ImGui::Button("Spray")) {
      *currentMode = MODE_SPRAY_INTERPOLATION;
      sprayInterpolationTool->initSketch();
    }
    ImGui::SameLine();
  }
  if (ImGui::Button("Delete")) {
    *currentMode = MODE_DELETION;
    deleteTool->initSketch();
  }

  // Surface Selection
  ImGui::Text("\nSurface Selection:");
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("Pseudo Surface", currentSurfaceMode, SURFACE_MODE_PSEUDO);
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("Greedy Surface", currentSurfaceMode, SURFACE_MODE_GREEDY);

  polyscope::SurfaceMesh *pseudoSurface = polyscope::getSurfaceMesh(PseudoSurfaceName);
  polyscope::SurfaceMesh *greedySurface = polyscope::getSurfaceMesh(GreedyProjName);
  switch (*currentSurfaceMode) {
    case SURFACE_MODE_PSEUDO:
      pseudoSurface->setEnabled(true);
      greedySurface->setEnabled(false);
      break;
    case SURFACE_MODE_GREEDY:
      pseudoSurface->setEnabled(false);
      greedySurface->setEnabled(true);
      break;
  }
}