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
  SURFACE_MODE_PSEUDO,
  SURFACE_MODE_GREEDY,
};

struct GuiManager {
  public:
    GuiManager() {}
    GuiManager(
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
      *currentSurfaceMode = SURFACE_MODE_PSEUDO;
    }

    ~GuiManager() {}

    // Window handlers
    void enableAdminToolWindow();   // Admin Tool Window
    void enableEditingToolWindow(); // Editing Tool Window
    void enableLogWindow();         // Log Window
  
  private:
    // Button Manager
    int *currentMode;
    int *currentSurfaceMode;

    // PointCloud
    PointCloud* pointCloud;

    // Editing Tools
    SketchInterpolationTool*  sketchInterpolationTool;
    SprayInterpolationTool*   sprayInterpolationTool;
    DeleteTool*               deleteTool;

    // Window Size
    int AdminToolWindowWidth = 300;
    int AdminToolWindowHeight = 150;
    int MarginAdminEditing = 50;
    int EditingToolWindowWidth = 300;
    int EditingToolWindowHeight = 400;
    int LogWindowHeight = 300;
    int LogWindowWidth = 400;

    // Variables for User Study
    clock_t   start_clock;            // Clock when the task was started
    bool      exportedLog = false;    // Whether already exported log. Log is exporeted only once.
};