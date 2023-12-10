#include "polyscope/polyscope.h"

#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"

#include "modes.hpp"
#include "surface.hpp"
#include "constants.hpp"

void ModeSelector::enableModeSelection(ImGuiIO &io) {
  // Undo/Redo
  ImGui::Text("Undo/Redo:");
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
      *currentSurfaceMode = SURFACE_MODE_PSEUDO;
      break;
    case MODE_INTERPOLATION_PAINT:
      ImGui::Text("   Selected Tool: Paint-Interpolation Tool");
      mlsSprayTool->launchToolOperation();
      *currentSurfaceMode = SURFACE_MODE_PSEUDO;
      break;
    case MODE_DELETION_SKETCH:
      ImGui::Text("   Selected Tool: Sketch-Deletion Tool");
      deleteTool->launchToolOperation();
      *currentSurfaceMode = SURFACE_MODE_PSEUDO;
      break;
    case MODE_DELETION_PAINT:
      ImGui::Text("   Selected Tool: Paint-Deletion Tool");
      deleteSprayTool->launchToolOperation();
      *currentSurfaceMode = SURFACE_MODE_PSEUDO;
      break;
  }

  // Surface Reconstruction
  ImGui::Text("\nSurface Reconstruction:");
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Reconstrut Poisson Surface")) {
    Surface poissonSurface(PoissonName, pointCloud->getVertices(), pointCloud->getNormals());
    poissonSurface.reconstructPoissonSurface(pointCloud->getAverageDistance());
    *currentSurfaceMode = SURFACE_MODE_POISSON;
  }

  // Surface Visualization
  ImGui::Text("\nSurface Visualization:");
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("None", currentSurfaceMode, SURFACE_MODE_NONE);
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("Pseudo Surface", currentSurfaceMode, SURFACE_MODE_PSEUDO);
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("Poisson Surface", currentSurfaceMode, SURFACE_MODE_POISSON);

  polyscope::SurfaceMesh *pseudoSurface = polyscope::getSurfaceMesh(PseudoSurfaceName);
  polyscope::SurfaceMesh *poissonSurface = polyscope::getSurfaceMesh(PoissonName);
  switch (*currentSurfaceMode) {
    case SURFACE_MODE_NONE:
      pseudoSurface->setEnabled(false);
      poissonSurface->setEnabled(false);
      break;
    case SURFACE_MODE_PSEUDO:
      pseudoSurface->setEnabled(true);
      poissonSurface->setEnabled(false);
      break;
    case SURFACE_MODE_POISSON:
      pseudoSurface->setEnabled(false);
      poissonSurface->setEnabled(true);
      break;
  }
}