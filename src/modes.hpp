#pragma once

#include "polyscope/polyscope.h"

enum Mode {
  MODE_NONE,
  MODE_TRACE,
  MODE_SPHERE_CAST,
  MODE_PATCH,
};

struct ModeSelector {
  public:
    void enableModeSelection(ImGuiIO &io);
  
  private:
    Mode currentMode;
};