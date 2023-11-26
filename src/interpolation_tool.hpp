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
    //  - Cast reconstructed surface onto the screen plane.
    //  - Filter only the points inside of the sketch.
    //  - Uniform the density of interpolated points.
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> filterSurfacePoints(
      Eigen::MatrixXd &surfacePoints,
      Eigen::MatrixXi &surfaceFaces
    );
};