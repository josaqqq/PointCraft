#pragma once

#include "polyscope/polyscope.h"

#include "point_cloud.hpp"
#include "sketch_tool.hpp"
#include "interpolation_tool.hpp"
#include "mls_spray_tool.hpp"
#include "delete_tool.hpp"
#include "delete_spray_tool.hpp"

enum Mode {
  MODE_NONE,
  MODE_INTERPOLATION_SKETCH,
  MODE_DELETION_SKETCH,
  MODE_INTERPOLATION_PAINT,
  MODE_DELETION_PAINT,
};

enum VisualizationMode {
  MODE_HIDE,
  MODE_SHOW
};

enum SurfaceMode {
  SURFACE_MODE_NONE,
  SURFACE_MODE_PSEUDO,
  SURFACE_MODE_POISSON,
  SURFACE_MODE_GREEDY,
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
      DeleteTool *deleteTool,
      DeleteSprayTool *deleteSprayTool
    ) 
    : currentMode(currentMode),
      currentSurfaceMode(currentSurfaceMode),

      pointCloud(pointCloud), 
      
      interpolationTool(interpolationTool),
      mlsSprayTool(mlsSprayTool),
      deleteTool(deleteTool),
      deleteSprayTool(deleteSprayTool)
    { 
      // Initialize current modes.
      *currentMode = MODE_NONE;
      *currentSurfaceMode = SURFACE_MODE_GREEDY;
    }

    ~ModeSelector() {}

    void enableModeSelection(ImGuiIO &io);
  
  private:
    int *currentMode;
    int *currentSurfaceMode;

    // Variables that control the version of the point cloud 
    // used for the final surface reconstruction.
    int lastVersionPseudo = 0;
    int lastVersionPoisson = 0;
    int lastVersionGreedy = 0;

    PointCloud        *pointCloud;

    InterpolationTool *interpolationTool;
    MLSSprayTool      *mlsSprayTool;
    DeleteTool        *deleteTool;
    DeleteSprayTool   *deleteSprayTool;
};