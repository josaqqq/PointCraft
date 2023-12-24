#include "polyscope/polyscope.h"

#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"

#include <fstream>

#include "gui.hpp"
#include "surface.hpp"
#include "constants.hpp"

// Admin Tool Window
void GuiManager::enableAdminToolWindow() {
  if (exportedLog) return;

  const int WindowWidth = polyscope::view::windowWidth;
  ImGui::SetNextWindowPos(ImVec2(WindowWidth - AdminToolWindowWidth, 0));
  ImGui::SetNextWindowSize(ImVec2(AdminToolWindowWidth, AdminToolWindowHeight));
  ImGui::Begin("Admin Tool");

  // Start Clock
  if (!startedClock) {
    if (ImGui::Button("Start Timer")) {
      polyscope::view::resetCameraToHomeView();
      start_clock = std::chrono::high_resolution_clock::now();
      startedClock = true;
    }
    ImGui::Text("");
  } else {
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 0, 0, 255));
    ImGui::Text("Timer Started.");
    ImGui::PopStyleColor();
    ImGui::Text("");
  }

  // Export .obj file
  if (debugMode) {
    if (ImGui::Button("Export .obj file")) {
      pointCloud->exportOBJFile("output");
    }
  }

  ImGui::End();
}

// Editing Tool Window
void GuiManager::enableEditingToolWindow() {
  if (exportedLog) {
    polyscope::view::moveScale = 1.0;
    return;
  }

  const int WindowWidth = polyscope::view::windowWidth;
  ImGui::SetNextWindowPos(ImVec2(WindowWidth - EditingToolWindowWidth, AdminToolWindowHeight + MarginAdminEditing));
  ImGui::SetNextWindowSize(ImVec2(EditingToolWindowWidth, EditingToolWindowHeight));
  ImGui::Begin("Editing Tool");

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

  if (sprayInterpolationTool != nullptr) {
    if (ImGui::Button("Spray")) {
      *currentMode = MODE_SPRAY_INTERPOLATION;
      sprayInterpolationTool->initSketch();
    }
    ImGui::SameLine();
  }
  if (sketchInterpolationTool != nullptr) {
    if (ImGui::Button("Sketch")) {
      *currentMode = MODE_SKETCH_INTERPOLATION;
      sketchInterpolationTool->initSketch();
    }
    ImGui::SameLine();
  }
  if (debugMode) {
    if (ImGui::Button("Delete")) {
      *currentMode = MODE_DELETION;
      deleteTool->initSketch();
    }
  }

  // Surface Selection
  polyscope::PointCloud *points = polyscope::getPointCloud(PointName);
  polyscope::SurfaceMesh *pseudoSurface = polyscope::getSurfaceMesh(PseudoSurfaceName);
  polyscope::SurfaceMesh *greedySurface = polyscope::getSurfaceMesh(GreedyProjName);
  if (debugMode) {
    ImGui::Text("\nSurface Selection:");
    ImGui::Text("   "); ImGui::SameLine();
    ImGui::RadioButton("Point Cloud", currentSurfaceMode, SURFACE_MODE_POINT);
    ImGui::Text("   "); ImGui::SameLine();
    ImGui::RadioButton("Pseudo Surface", currentSurfaceMode, SURFACE_MODE_PSEUDO);
    ImGui::Text("   "); ImGui::SameLine();
    ImGui::RadioButton("Reconstructed Mesh", currentSurfaceMode, SURFACE_MODE_GREEDY);

    switch (*currentSurfaceMode) {
      case SURFACE_MODE_POINT:
        points->setEnabled(true);
        pseudoSurface->setEnabled(false);
        pseudoSurface->setEnabled(false);
        break;
      case SURFACE_MODE_PSEUDO:
        points->setEnabled(false);
        pseudoSurface->setEnabled(true);
        greedySurface->setEnabled(false);
        break;
      case SURFACE_MODE_GREEDY:
        points->setEnabled(false);
        pseudoSurface->setEnabled(false);
        greedySurface->setEnabled(true);
        break;
    }
  } else {
    pseudoSurface->setEnabled(true);
    greedySurface->setEnabled(false);
  }

  ImGui::End();
}

// Log Window
void GuiManager::enableLogWindow() {
  if (pointCloud->getBoundaryPointNum() != 0) return;

  // Stop timer
  auto end_clock = std::chrono::high_resolution_clock::now();

  // If not exported the log yet, export the log
  if (!exportedLog) {
    // Determine log file name
    int lastSlashIdx = -1;
    for (size_t i = 0; i < inputFile.size(); i++) {
      if (inputFile[i] == '/') lastSlashIdx = i;
    }
    std::string logFileName = inputFile.substr(lastSlashIdx + 1);
    if (sprayInterpolationTool != nullptr) logFileName += "__spray";
    if (sketchInterpolationTool != nullptr) logFileName += "__sketch";
    logFileName += "__user-" + std::to_string(userID);

    // Log elapsed time
    std::ofstream logFile(logFileName);
    auto totalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(end_clock - start_clock);
    logFile << "Total Elapsed Time (ms):\n" << totalDuration.count() << '\n';
    logFile.close();

    // Write logs to logFile (Add lines to the end of the file)
    pointCloud->exportLog(start_clock, logFileName);
    if (sprayInterpolationTool != nullptr)  sprayInterpolationTool->exportLog(start_clock, logFileName, "Spray Interpolation Tool");
    if (sketchInterpolationTool != nullptr) sketchInterpolationTool->exportLog(start_clock, logFileName, "Sketch Interpolation Tool");
    deleteTool->exportLog(start_clock, logFileName, "Delete Tool");
    
    std::cout << "Exported " << logFileName << std::endl;

    // Export completed point cloud
    pointCloud->exportOBJFile(logFileName);
  }
  
  // Set exportedLog flag
  exportedLog = true;

  // Render Log Window
  const int WindowWidth = polyscope::view::windowWidth;
  const int WindowHeight = polyscope::view::windowHeight;
  ImGui::SetNextWindowPos(ImVec2(WindowWidth/2 - LogWindowWidth/2, WindowHeight/2 - LogWindowHeight/2));
  ImGui::SetNextWindowSize(ImVec2(LogWindowWidth, LogWindowHeight));
  ImGui::Begin("Log Window");
  ImGui::Text("Task Completed!!");
  ImGui::End();
}