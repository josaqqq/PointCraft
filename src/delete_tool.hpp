#pragma once

#include <string>

#include "sketch_tool.hpp"
#include "constants.hpp"

class DeleteTool: public SketchTool {
  public:
    DeleteTool(int *currentMode, PointCloud *pointCloud)
    : SketchTool(currentMode, pointCloud) {
      int* surfacePointNumPtr = getSurfacePointNumPtr();
      *surfacePointNumPtr = DeleteSurfacePointNum;
    }
    ~DeleteTool() {}

    void launchToolOperation() override;

  private:
    void draggingEvent();
    void releasedEvent();
};