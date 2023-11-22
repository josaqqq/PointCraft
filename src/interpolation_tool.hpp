#pragma once

#include "sketch_tool.hpp"

class InterpolationTool : public SketchTool {
  public:
    InterpolationTool(PointCloud *pointCloud, int *currentMode)
    : SketchTool(pointCloud, currentMode) {}
    ~InterpolationTool() {}

    void drawSketch() override;

  private:
    void draggingEvent();
    void releasedEvent();

    void renderInterpolatedPoints(
      Eigen::MatrixXd &newV,
      Eigen::MatrixXd &newN
    );
};