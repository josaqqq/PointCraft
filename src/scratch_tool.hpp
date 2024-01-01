#pragma once

#include "sketch_tool.hpp"
#include "constants.hpp"

class ScratchTool: public SketchTool {
  public:
    ScratchTool(bool *enableSurfacePoints, int *currentMode, PointCloud *pointCloud)
    : SketchTool(enableSurfacePoints, currentMode, pointCloud) {
      int* surfacePointNumPtr = getSurfacePointNumPtr();
      *surfacePointNumPtr = DeleteSurfacePointNum;
    }
    ~ScratchTool() {}

    void launchToolOperation() override;

  private:
    void draggingEvent();
    void releasedEvent();
};