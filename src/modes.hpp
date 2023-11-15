#pragma once

#include "polyscope/polyscope.h"

enum Mode {
  MODE_NONE,
  MODE_TRACE,
  MODE_SPHERE_CAST,
  MODE_PATCH,
};

enum SurfaceMode {
  SURFACE_MODE_NONE,
  SURFACE_MODE_POISSON,
  SURFACE_MODE_GREEDY,
};

struct ModeSelector {
  public:
    ModeSelector() {
      // Initialize current modes.
      currentMode = MODE_NONE;
      currentSurfaceMode = SURFACE_MODE_GREEDY;
    }
    void enableModeSelection(ImGuiIO &io);
  
  private:
    int currentMode;
    int currentSurfaceMode;
};