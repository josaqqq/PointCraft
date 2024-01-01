#pragma once

#include "sketch_tool.hpp"
#include "constants.hpp"

class SketchInterpolationTool : public SketchTool {
  public:
    SketchInterpolationTool(bool *enableSurfacePoints, int *currentMode, PointCloud *pointCloud)
    : SketchTool(enableSurfacePoints, currentMode, pointCloud) {
      int* surfacePointNumPtr = getSurfacePointNumPtr();
      *surfacePointNumPtr = SketchSurfacePointNum;
    }
    ~SketchInterpolationTool() {}

    void launchToolOperation() override;

  private:
    void draggingEvent();
    void releasedEvent();

    // Register new vertices and normals as point cloud
    void renderInterpolatedPoints(
      std::vector<glm::dvec3> &newV,
      std::vector<glm::dvec3> &newN
    );

    // Filter the interpolated points with depth
    //  1. Cast interpolated points onto the screen plane.
    //  2. Construct octree for the casted interpolated points.
    //  3. Search for a candidate point for each discretized grid.
    //    - Only points that their normals are directed to cameraOrig.
    //  4. Detect depth with DBSCAN
    //
    //  - surfacePoints:  Positions of the interpolated surface points
    //  - surfaceNormals: Normals of the interpolated surface points
    std::set<int> filterWithDepth(
      std::vector<glm::dvec3> &surfacePoints,
      std::vector<glm::dvec3> &surfaceNormals
    );
};