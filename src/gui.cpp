#include "polyscope/polyscope.h"

#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"

#include "gui.hpp"
#include "surface.hpp"
#include "constants.hpp"

// Admin Tool Window
void GuiManager::enableAdminToolWindow() {
  const int WindowWidth = polyscope::view::windowWidth;
  ImGui::SetNextWindowPos(ImVec2(WindowWidth - AdminToolWindowWidth, 0));
  ImGui::SetNextWindowSize(ImVec2(AdminToolWindowWidth, AdminToolWindowHeight));
  ImGui::Begin("Admin Tool");

  // Start Clock
  if (ImGui::Button("Start timer")) {
    start_clock = clock();
  }
  ImGui::Text("");

  // Export .obj file
  if (ImGui::Button("Export .obj file")) {
    pointCloud->exportOBJFile();
  }

  ImGui::End();
}

// Editing Tool Window
void GuiManager::enableEditingToolWindow() {
  const int WindowWidth = polyscope::view::windowWidth;
  ImGui::SetNextWindowPos(ImVec2(WindowWidth - EditingToolWindowWidth, AdminToolWindowHeight + MarginAdminEditing));
  ImGui::SetNextWindowSize(ImVec2(EditingToolWindowWidth, EditingToolWindowHeight));
  ImGui::Begin("Editing Tool");

  // Undo/Redo
  ImGui::Text("Undo/Redo:");
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

  ImGui::End();
}

// Log Window
void GuiManager::enableLogWindow() {
  const int WindowWidth = polyscope::view::windowWidth;
  const int WindowHeight = polyscope::view::windowHeight;
  ImGui::SetNextWindowPos(ImVec2(WindowWidth/2 - LogWindowWidth/2, WindowHeight/2 - LogWindowHeight/2));
  ImGui::SetNextWindowSize(ImVec2(LogWindowWidth, LogWindowHeight));
  ImGui::Begin("Log Window");

  ImGui::Text("Log Window");

  ImGui::End();
}