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
      Eigen::MatrixXd &newV,
      Eigen::MatrixXd &newN
    );

    // Filter the reconstructed surface
    //  - Cast reconstructed surface and basisPoints onto the screen plane.
    //  - Construct octree for the casted surface points and basisPoints.
    //  - Search for a candidate point for each discretized grid.
    //    - Only points that their normals are directed to cameraOrig.
    //    - If the candidate point is one of basisPoints, then skip it.
    //  - Detect depth with DBSCAN
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> filterSurfacePoints(
      Eigen::MatrixXd &surfacePoints,
      Eigen::MatrixXi &surfaceFaces
    );
};