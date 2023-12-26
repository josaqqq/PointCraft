#include "polyscope/polyscope.h"

#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"

#include <fstream>

#include "gui.hpp"
#include "surface.hpp"
#include "constants.hpp"

// Admin Tool Window
void GuiManager::enableAdminToolWindow() {
  const int WindowWidth = polyscope::view::windowWidth;
  ImGui::SetNextWindowPos(ImVec2(WindowWidth - AdminToolWindowWidth, 0));
  ImGui::SetNextWindowSize(ImVec2(AdminToolWindowWidth, AdminToolWindowHeight));
  ImGui::Begin("Admin Tool Window");

  // Export .obj file
  if (ImGui::Button("Export .obj file")) {
    pointCloud->exportOBJFile("output");
  }
  ImGui::Text("");

  // Enable Surface Visualization
  ImGui::Checkbox("Enable Surface Points", enableSurfacePoints);

  // Render Mode Selection
  ImGui::Text("\nRender Mode Selection:");
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("Pseudo Surface", currentSurfaceMode, RENDER_MODE_PSEUDO);
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("Point Cloud", currentSurfaceMode, RENDER_MODE_POINT);

  polyscope::SurfaceMesh *pseudoSurfacePtr = polyscope::getSurfaceMesh(PseudoSurfaceName);
  polyscope::PointCloud *pointCloudPtr = polyscope::getPointCloud(PointName);
  switch (*currentSurfaceMode) {
    case RENDER_MODE_PSEUDO:
      pointCloudPtr->setEnabled(false);
      pseudoSurfacePtr->setEnabled(true);
      break;
    case RENDER_MODE_POINT:
      pointCloudPtr->setEnabled(true);
      pseudoSurfacePtr->setEnabled(false);
      break;
  }

  ImGui::End();
}

// Editing Tool Window
void GuiManager::enableEditingToolWindow() {
  const int WindowWidth = polyscope::view::windowWidth;
  ImGui::SetNextWindowPos(ImVec2(WindowWidth - EditingToolWindowWidth, AdminToolWindowHeight + MarginAdminEditing));
  ImGui::SetNextWindowSize(ImVec2(EditingToolWindowWidth, EditingToolWindowHeight));
  ImGui::Begin("Editing Tool Window");

  // Undo/Redo
  if (ImGui::Button("<< Undo")) pointCloud->executeUndo();
  ImGui::SameLine();
  if (ImGui::Button("Redo >>")) pointCloud->executeRedo();
  ImGui::Text("");

  // Tool Selection
  switch (*currentMode) {
    case MODE_NONE:
      ImGui::Text("Selected Tool: None");
      polyscope::view::moveScale = 1.0;
      break;
    case MODE_SKETCH_INTERPOLATION:
      ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 255, 0, 255));
      ImGui::Text("Selected Tool: Sketch");
      ImGui::PopStyleColor();
      sketchInterpolationTool->launchToolOperation();
      break;
    case MODE_SPRAY_INTERPOLATION:
      ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 255, 0, 255));
      ImGui::Text("Selected Tool: Spray");
      ImGui::PopStyleColor();
      sprayInterpolationTool->launchToolOperation();
      break;
    case MODE_DELETION:
      ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 255, 0, 255));
      ImGui::Text("Selected Tool: Delete");
      ImGui::PopStyleColor();
      deleteTool->launchToolOperation();
      break;
  }

  if (ImGui::Button("Spray")) {
    *currentMode = MODE_SPRAY_INTERPOLATION;
    sprayInterpolationTool->initSketch();
  }
  ImGui::SameLine();
  if (ImGui::Button("Sketch")) {
    *currentMode = MODE_SKETCH_INTERPOLATION;
    sketchInterpolationTool->initSketch();
  }
  ImGui::SameLine();
  if (ImGui::Button("Delete")) {
    *currentMode = MODE_DELETION;
    deleteTool->initSketch();
  }

  ImGui::End();
}
