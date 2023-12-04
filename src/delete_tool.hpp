#pragma once

#include "sketch_tool.hpp"

class DeleteTool: public SketchTool {
  public:
    DeleteTool(int *currentMode, PointCloud *pointCloud)
    : SketchTool(currentMode, pointCloud) {}
    ~DeleteTool() {}

    void launchToolOperation() override;

  private:
    void draggingEvent();
    void releasedEvent();
};