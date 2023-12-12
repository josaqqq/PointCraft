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

    // Calculate normals of the surface points
    std::vector<glm::dvec3> calculateSurfaceNormals(
      std::vector<glm::dvec3> &surfacePoints,
      std::vector<std::vector<size_t>> &surfaceFaces
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