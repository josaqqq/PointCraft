#pragma once

#include "sketch_tool.hpp"
#include "constants.hpp"

class FeatureTool: public SketchTool {
  public:
    FeatureTool(bool *enableSurfacePoints, int *currentMode, PointCloud *pointCloud)
    : SketchTool(enableSurfacePoints, currentMode, pointCloud) {
      int *surfacePointNumPtr = getSurfacePointNumPtr();
      *surfacePointNumPtr = FeatureSurfacePointNum;
    }
    ~FeatureTool() {}

    void launchToolOperation() override;

  private:
    void draggingEvent();
    void releasedEvent();

    int poissonNum = 0;
};