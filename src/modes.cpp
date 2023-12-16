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
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Reset Tool")) {
    *currentMode = MODE_NONE;
  }
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Sketch-Interpolation Tool")) {
    *currentMode = MODE_INTERPOLATION_SKETCH;
    interpolationTool->initSketch();
  }
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Paint-Interpolation Tool")) {
    *currentMode = MODE_INTERPOLATION_PAINT;
    mlsSprayTool->initSketch();
  }
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Sketch-Deletion Tool")) {
    *currentMode = MODE_DELETION_SKETCH;
    deleteTool->initSketch();
  }
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Paint-Deletion Tool")) {
    *currentMode = MODE_DELETION_PAINT;
    deleteSprayTool->initSketch();
  }

  switch (*currentMode) {
    case MODE_NONE:
      ImGui::Text("   Selected Tool: None");
      polyscope::view::moveScale = 1.0;
      break;
    case MODE_INTERPOLATION_SKETCH:
      ImGui::Text("   Selected Tool: Sketch-Interpolation Tool");
      interpolationTool->launchToolOperation();
      break;
    case MODE_INTERPOLATION_PAINT:
      ImGui::Text("   Selected Tool: Paint-Interpolation Tool");
      mlsSprayTool->launchToolOperation();
      break;
    case MODE_DELETION_SKETCH:
      ImGui::Text("   Selected Tool: Sketch-Deletion Tool");
      deleteTool->launchToolOperation();
      break;
    case MODE_DELETION_PAINT:
      ImGui::Text("   Selected Tool: Paint-Deletion Tool");
      deleteSprayTool->launchToolOperation();
      break;
  }

  // Surface Visualization
  ImGui::Text("\nSurface Visualization:");
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("None", currentSurfaceMode, SURFACE_MODE_NONE);
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("Pseudo Surface", currentSurfaceMode, SURFACE_MODE_PSEUDO);
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("Greedy Surface", currentSurfaceMode, SURFACE_MODE_GREEDY);

  int currentVersion = pointCloud->getCurrentVersion();
  polyscope::SurfaceMesh *pseudoSurface;
  polyscope::SurfaceMesh *greedySurface;
  switch (*currentSurfaceMode) {
    case SURFACE_MODE_NONE:
      // Control whether enabled or not
      pseudoSurface = polyscope::getSurfaceMesh(PseudoSurfaceName);
      greedySurface = polyscope::getSurfaceMesh(GreedyProjName);
      pseudoSurface->setEnabled(false);
      greedySurface->setEnabled(false);
      break;
    case SURFACE_MODE_PSEUDO:
      if (lastVersionPseudo != currentVersion) {
        // Show Pseudo Surface
        Surface pseudoSurface(PseudoSurfaceName, pointCloud->getVertices(), pointCloud->getNormals());
        pseudoSurface.showPseudoSurface(pointCloud->getAverageDistance(), false);
        lastVersionPseudo = currentVersion;
      }

      // Control whether enabled or not
      pseudoSurface = polyscope::getSurfaceMesh(PseudoSurfaceName);
      greedySurface = polyscope::getSurfaceMesh(GreedyProjName);
      pseudoSurface->setEnabled(true);
      greedySurface->setEnabled(false);
      break;
    case SURFACE_MODE_GREEDY:
      if (lastVersionGreedy != currentVersion) {
        // Show Greedy Surface
        Surface greedySurfacee(GreedyProjName, pointCloud->getVertices(), pointCloud->getNormals());
        greedySurfacee.showGreedyProjection(pointCloud->getAverageDistance(), false);
        lastVersionGreedy = currentVersion;
      }

      // Control whether enabled or not
      pseudoSurface = polyscope::getSurfaceMesh(PseudoSurfaceName);
      greedySurface = polyscope::getSurfaceMesh(GreedyProjName);
      pseudoSurface->setEnabled(false);
      greedySurface->setEnabled(true);
      break;
  }
}