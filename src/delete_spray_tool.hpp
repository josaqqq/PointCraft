#pragma once

#include "sketch_tool.hpp"

class DeleteSprayTool: public SketchTool {
  public:
    DeleteSprayTool(int *currentMode, PointCloud *pointCloud)
    : SketchTool(currentMode, pointCloud) {}
    ~DeleteSprayTool() {}

    void launchToolOperation() override;

  private:
    void draggingEvent();
    void releasedEvent();
};
