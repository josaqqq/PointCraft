#pragma once

#include "polyscope/polyscope.h"

#include "point_cloud.hpp"
#include "sketch_tool.hpp"
#include "interpolation_tool.hpp"
#include "mls_spray_tool.hpp"
#include "delete_tool.hpp"

enum Mode {
  MODE_NONE,
  MODE_INTERPOLATION,
  MODE_DELETE,
  MODE_MLS_SPRAY,
};

enum VisualizationMode {
  MODE_HIDE,
  MODE_SHOW
};

enum SurfaceMode {
  SURFACE_MODE_NONE,
  SURFACE_MODE_PSEUDO,
  SURFACE_MODE_POISSON,
};

struct ModeSelector {
  public:
    ModeSelector() {}
    ModeSelector(
      int *currentMode,
      int *currentSurfaceMode,

      PointCloud *pointCloud,
      
      InterpolationTool *interpolationTool,
      MLSSprayTool *mlsSprayTool,
      DeleteTool *deleteTool
    ) 
    : currentMode(currentMode),
      currentSurfaceMode(currentSurfaceMode),

      pointCloud(pointCloud), 
      
      interpolationTool(interpolationTool),
      mlsSprayTool(mlsSprayTool),
      deleteTool(deleteTool)
    { 
      // Initialize current modes.
      *currentMode = MODE_NONE;
      *currentSurfaceMode = SURFACE_MODE_PSEUDO;
    }

    ~ModeSelector() {}

    void enableModeSelection(ImGuiIO &io);
  
  private:
    int *currentMode;
    int *currentSurfaceMode;

    PointCloud        *pointCloud;

    InterpolationTool *interpolationTool;
    MLSSprayTool      *mlsSprayTool;
    DeleteTool        *deleteTool;
};