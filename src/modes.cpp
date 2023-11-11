#include "polyscope/polyscope.h"

#include "modes.hpp"
#include "patch.hpp"

void ModeSelector::enableModeSelection(ImGuiIO &io) {
  ImGui::Text("Mode Selection:");
  if (ImGui::Button("Trace Mode")) {
    currentMode = MODE_TRACE;
  }
  if (ImGui::Button("Sphere Cast Mode")) {
    currentMode = MODE_SPHERE_CAST;
  }

  switch (currentMode) {
    case MODE_NONE:
      ImGui::Text("Selected Mode: None");
      polyscope::view::moveScale = 1.0;
      break;

    case MODE_TRACE:
      ImGui::Text("Selected Mode: Trace Mode");
      polyscope::view::moveScale = 0.0;
      tracePoints(io, currentMode);
      break;

    case MODE_SPHERE_CAST:
      ImGui::Text("Selected Mode: Sphere Cast Mode");
      polyscope::view::moveScale = 0.0;
      castPointToSphere(io, currentMode, glm::vec3(0.0, 0.0, 0.0), 1.0);
      break;
  }
}