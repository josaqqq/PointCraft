#pragma once

#include <vector>
#include "sketch_tool.hpp"

// "It randomly creates points inside the brush volume
// and projects them onto the MLS surface in th brush's vicinity."
//
// Weyrich, T., Pauly, M., Keiser, R., Heinzle, S., Scandella, S.,
// & Gross, M. H. (2004, June). Post-processing of Scanned 3D Surface Data.
// In PBG (pp. 85-94).
class MLSSprayTool : public SketchTool {
  public:
    MLSSprayTool(int *currentMode, PointCloud *pointCloud)
    : SketchTool(currentMode, pointCloud) {}
    ~MLSSprayTool() {}

    void launchToolOperation() override;

  private:
    void draggingEvent();
    void releasedEvent();
};