#pragma once

#include "sketch_tool.hpp"

class InterpolationTool : public SketchTool {
  public:
    InterpolationTool(int *currentMode, PointCloud *pointCloud)
    : SketchTool(currentMode, pointCloud) {}
    ~InterpolationTool() {}

    bool drawSketch() override;

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
    //  - Check the conditions below
    //    1. Judge inside/outside of the sketch and the convex hull of basisPoints.
    //    2. Check the normal direction of the point.
    //    3. Check the nearest neighbors' distances from cameraOrig
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> filterSurfacePoints(
      Eigen::MatrixXd &surfacePoints,
      Eigen::MatrixXi &surfaceFaces
    );
};