#pragma once

#include "polyscope/polyscope.h"

#include <chrono>

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
      std::string inputFile,
      int userID,
      bool debugMode,

      int *currentMode,
      int *currentSurfaceMode,

      PointCloud *pointCloud,
      
      SketchInterpolationTool *sketchInterpolationTool,
      SprayInterpolationTool  *sprayInterpolationTool,
      DeleteTool              *deleteTool
    ) 
    : inputFile(inputFile),
      userID(userID),
      debugMode(debugMode),

      currentMode(currentMode),
      currentSurfaceMode(currentSurfaceMode),

      pointCloud(pointCloud), 
      
      sketchInterpolationTool(sketchInterpolationTool),
      sprayInterpolationTool(sprayInterpolationTool),
      deleteTool(deleteTool)
    { 
      // Initialize current modes.
      *currentMode = MODE_NONE;
      *currentSurfaceMode = SURFACE_MODE_PSEUDO;

      // Initialize start_clock
      start_clock = std::chrono::high_resolution_clock::now();
    }

    ~GuiManager() {}

    // Window handlers
    void enableAdminToolWindow();   // Admin Tool Window
    void enableEditingToolWindow(); // Editing Tool Window
    void enableLogWindow();         // Log Window
  
  private:
    // User Study Environment
    std::string inputFile;
    int userID;
    bool debugMode;

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
    int LogWindowWidth = 300;
    int LogWindowHeight = 100;

    // Variables for User Study
    std::chrono::high_resolution_clock::time_point start_clock; // Clock when the task was started
    bool exportedLog = false;    // When exported log -> exportedLog = true;
};