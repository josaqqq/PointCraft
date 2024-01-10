#pragma once

#include "polyscope/polyscope.h"

#include <chrono>

#include "point_cloud.hpp"
#include "sketch_tool.hpp"
#include "sketch_interpolation_tool.hpp"
#include "spray_interpolation_tool.hpp"
#include "feature_tool.hpp"
#include "scratch_tool.hpp"
#include "delete_tool.hpp"

enum Mode {
  MODE_NONE,
  MODE_SKETCH_INTERPOLATION,
  MODE_SPRAY_INTERPOLATION,
  MODE_FEATURE,
  MODE_SCRATCH,
  MODE_DELETION,
};

enum VisualizationMode {
  MODE_HIDE,
  MODE_SHOW,
};

enum RenderMode {
  RENDER_MODE_POINT,
  RENDER_MODE_PSEUDO,
  RENDER_MODE_SURFACE
};

struct GuiManager {
  public:
    GuiManager() {}
    GuiManager(
      std::string inputFile,
      bool debugMode,

      bool *enableSurfacePoints,
      bool *visualizeHoles,
      int *currentMode,
      int *currentSurfaceMode,

      PointCloud *pointCloud,
      
      SketchInterpolationTool *sketchInterpolationTool,
      SprayInterpolationTool  *sprayInterpolationTool,
      FeatureTool             *featureTool,
      ScratchTool             *scratchTool,
      DeleteTool              *deleteTool
    ) 
    : inputFile(inputFile),
      debugMode(debugMode),

      enableSurfacePoints(enableSurfacePoints),
      visualizeHoles(visualizeHoles),
      currentMode(currentMode),
      currentSurfaceMode(currentSurfaceMode),

      pointCloud(pointCloud), 
      
      sketchInterpolationTool(sketchInterpolationTool),
      sprayInterpolationTool(sprayInterpolationTool),
      featureTool(featureTool),
      scratchTool(scratchTool),
      deleteTool(deleteTool)
    { 
      // Initialize current modes.
      *enableSurfacePoints = false;
      *visualizeHoles = false;
      *currentMode = MODE_NONE;
      *currentSurfaceMode = RENDER_MODE_PSEUDO;
    }

    ~GuiManager() {}

    // Window handlers
    void enableAdminToolWindow();   // Admin Tool Window
    void enableEditingToolWindow(); // Editing Tool Window
  
  private:
    // User Study Environment
    std::string inputFile;
    bool debugMode;

    // Button Manager
    bool *enableSurfacePoints;
    bool *visualizeHoles;
    int *currentMode;
    int *currentSurfaceMode;

    // PointCloud
    PointCloud* pointCloud;

    // Editing Tools
    SketchInterpolationTool*  sketchInterpolationTool;
    SprayInterpolationTool*   sprayInterpolationTool;
    FeatureTool*              featureTool;
    ScratchTool*              scratchTool;
    DeleteTool*               deleteTool;

    // Window Size
    int AdminToolWindowWidth = 400;
    int AdminToolWindowHeight = 400;
    int MarginAdminEditing = 50;
    int EditingToolWindowWidth = 400;
    int EditingToolWindowHeight = 400;
    int LogWindowWidth = 400;
    int LogWindowHeight = 100;
};