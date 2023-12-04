#pragma once

#include "sketch_tool.hpp"

class InterpolationTool : public SketchTool {
  public:
    InterpolationTool(int *currentMode, PointCloud *pointCloud)
    : SketchTool(currentMode, pointCloud) {}
    ~InterpolationTool() {}

    void launchToolOperation() override;

  private:
    void draggingEvent();
    void releasedEvent();

    // Register new vertices and normals as point cloud
    void renderInterpolatedPoints(
      std::vector<glm::dvec3> &newV,
      std::vector<glm::dvec3> &newN
    );

    // Filter the reconstructed surface
    //  - Cast reconstructed surface and basisPoints onto the screen plane.
    //  - Construct octree for the casted surface points and basisPoints.
    //  - Search for a candidate point for each discretized grid.
    //    - Only points that their normals are directed to cameraOrig.
    //    - If the candidate point is one of basisPoints, then skip it.
    //  - Detect depth with DBSCAN
    std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> filterSurfacePoints(
      std::vector<glm::dvec3> &surfacePoints,
      std::vector<std::vector<size_t>> &surfaceFaces
    );
};