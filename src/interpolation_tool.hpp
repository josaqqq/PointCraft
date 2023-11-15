#pragma once

#include "sketch_tool.hpp"

class InterpolationTool : public SketchTool {
  public:
    InterpolationTool(PointCloud *pointCloud, int *currentMode)
    : SketchTool(pointCloud, currentMode) {}
    ~InterpolationTool() {}

    void drawSketch() override;

  private:
};