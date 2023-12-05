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
    //  1. Cast reconstructed surface and basisPoints onto the screen plane.
    //  2. Construct octree for the casted surface points and basisPoints.
    //  3. Search for a candidate point for each discretized grid.
    //    - Only points that their normals are directed to cameraOrig.
    //    - If the candidate point is one of basisPoints, then skip it.
    //  4. Detect depth with DBSCAN
    std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> filterSurfacePointsWithFaces(
      std::vector<glm::dvec3> &surfacePoints,
      std::vector<std::vector<size_t>> &surfaceFaces
    );
    std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> filterSurfacePointsWithNormals(
      std::vector<glm::dvec3> &surfacePoints,
      std::vector<glm::dvec3> &surfaceNormals
    );
    
    // Used in filterSurfacePointsWithFaces and filterSurfacePointsWithNormals
    // for  step-2, 3, 4
    //  - surfacePointsSize: the size of the points on the reconstructed surface
    //  - pointsInWorldCoord: the point coordinates of surface points and basis points in the world coordinate
    //  - normalsInWorldCoord: the normal vectors of surface points and basis points in the world coordinate
    //  - pontsCastedOntoScreen: the point coordinates of surface points and basis points in the local coordinate of the screen
    std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> filterSurfacePointsInner(
      int surfacePointsSize,
      std::vector<glm::dvec3> &pointsInWorldCoord,
      std::vector<glm::dvec3> &normalsInWorldCoord,
      std::vector<glm::dvec3> &pointsCastedOntoScreen
    );   
};