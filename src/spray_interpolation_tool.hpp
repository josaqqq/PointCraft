#pragma once

#include <vector>

#include "sketch_tool.hpp"
#include "constants.hpp"

// "It randomly creates points inside the brush volume
// and projects them onto the MLS surface in th brush's vicinity."
//
// Weyrich, T., Pauly, M., Keiser, R., Heinzle, S., Scandella, S.,
// & Gross, M. H. (2004, June). Post-processing of Scanned 3D Surface Data.
// In PBG (pp. 85-94).
class SprayInterpolationTool : public SketchTool {
  public:
    SprayInterpolationTool(bool *enableSurfacePoints, int *currentMode, PointCloud *pointCloud)
    : SketchTool(enableSurfacePoints, currentMode, pointCloud) {
      int* surfacePointNumPtr = getSurfacePointNumPtr();
      *surfacePointNumPtr = SpraySurfacePointNum;
    }
    ~SprayInterpolationTool() {}

    void launchToolOperation() override;

  private:
    void draggingEvent();
    void releasedEvent();
};