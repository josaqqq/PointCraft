#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include "modes.hpp"
#include "patch.hpp"
#include "surface.hpp"
#include "constants.hpp"

void ModeSelector::enableModeSelection(ImGuiIO &io) {
  // Surface Selection
  ImGui::Text("Surface Selection");
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("None", &currentSurfaceMode, SURFACE_MODE_NONE);
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("Poisson Surface", &currentSurfaceMode, SURFACE_MODE_POISSON);
  ImGui::Text("   "); ImGui::SameLine();
  ImGui::RadioButton("Greedy Surface", &currentSurfaceMode, SURFACE_MODE_GREEDY);

  polyscope::SurfaceMesh *poissonSurface = polyscope::getSurfaceMesh(PoissonName);
  polyscope::SurfaceMesh *greedySurface = polyscope::getSurfaceMesh(GreedyProjName);
  switch (currentSurfaceMode) {
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

  ImGui::Text("Mode Selection:");
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Reset Mode"))        currentMode = MODE_NONE;
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Trace Mode"))        currentMode = MODE_TRACE;
  // ImGui::Text("   "); ImGui::SameLine();
  // if (ImGui::Button("Sphere Cast Mode"))  currentMode = MODE_SPHERE_CAST;
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Patch Mode"))     currentMode = MODE_PATCH;

  switch (currentMode) {
    case MODE_NONE:
      ImGui::Text("   Selected Mode: None");
      polyscope::view::moveScale = 1.0;
      break;

    case MODE_TRACE:
      ImGui::Text("   Selected Mode: Trace Mode");
      polyscope::view::moveScale = 0.0;
      tracePoints(io, currentMode);
      break;

    case MODE_SPHERE_CAST:
      ImGui::Text("   Selected Mode: Sphere Cast Mode");
      polyscope::view::moveScale = 0.0;
      castPointToSphere(io, currentMode, glm::vec3(0.0, 0.0, 0.0), 1.0);
      break;
    
    case MODE_PATCH:
      ImGui::Text("   Selected Mode: Patch Mode");
      polyscope::view::moveScale = 0.0;
      createPatchToPointCloud(io, currentMode, glm::vec3(0.0, 0.0, 0.0), 1.0);
      break;
  }
}