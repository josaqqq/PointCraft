#pragma once

#include "polyscope/polyscope.h"

#include "point_cloud.hpp"
#include "sketch_tool.hpp"
#include "sketch_interpolation_tool.hpp"
#include "spray_interpolation_tool.hpp"
#include "delete_tool.hpp"

enum Mode {
  MODE_NONE,
  MODE_SKETCH_INTERPOLATION,
  MODE_SPRAY_INTERPOLATION,
  MODE_DELETION,
};

enum VisualizationMode {
  MODE_HIDE,
  MODE_SHOW
};

enum SurfaceMode {
  SURFACE_MODE_NONE,
  SURFACE_MODE_PSEUDO,
  SURFACE_MODE_GREEDY,
};

struct ModeSelector {
  public:
    ModeSelector() {}
    ModeSelector(
      int *currentMode,
      int *currentSurfaceMode,

      PointCloud *pointCloud,
      
      SketchInterpolationTool *sketchInterpolationTool,
      SprayInterpolationTool  *sprayInterpolationTool,
      DeleteTool              *deleteTool
    ) 
    : currentMode(currentMode),
      currentSurfaceMode(currentSurfaceMode),

      pointCloud(pointCloud), 
      
      sketchInterpolationTool(sketchInterpolationTool),
      sprayInterpolationTool(sprayInterpolationTool),
      deleteTool(deleteTool)
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

    SketchInterpolationTool *sketchInterpolationTool;
    SprayInterpolationTool  *sprayInterpolationTool;
    DeleteTool              *deleteTool;
};