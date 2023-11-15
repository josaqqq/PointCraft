#pragma once

#include "polyscope/polyscope.h"

#include "ray.hpp"
#include "point_cloud.hpp"

class SketchTool {
  public:
    SketchTool(PointCloud *pointCloud, int *currentMode) 
    : pointCloud(pointCloud), currentMode(currentMode) {}
    virtual ~SketchTool() {}

    // Draw sketch according to the selected mode.
    virtual void drawSketch() {};

    // Register/Remove sketch as curve network line with sketchName.
    // Be aware that the curve network with 
    // the same name is overwritten
    void registerSketchAsCurveNetworkLine(std::string sketchName);
    void removeSketchAsCurveNetworkLine(std::string sketchName);

    // Register/Remove sketch as curve network loop with sketchName.
    // Be aware that the curve network with 
    // the same name is overwritten
    void registerSketchAsCurveNetworkLoop(std::string sketchName);
    void removeSketchAsCurveNetworkLoop(std::string sketchName);

    // Select the boundary points the user selected
    // and update boundaryPoints.
    bool addBoundaryPoints(Hit hitInfo);

    // Cast the boundary points to the plane orthogonal to camera direction
    // and update boundaryOnPlane.
    void castBoundaryToCameraPlane();

    // Cast the boundary points to the screen and update boundaryOnPlane.
    void castBoundaryToScreen();

    // Discretize the boundary and update discretiedPoints.
    void discretizeCastedBoundary();

    // Reset all member variables.
    void resetSketch();

    // Return the size of boundaryPoints.
    int getBoundarySize();

    // Return the pointer to pointCloud.
    PointCloud* getPointCloud();

  private:
    PointCloud *pointCloud;

    int *currentMode;     // Current selected Mode
    double averageDepth;  // Average depth of selected points
    // std::unordered_set<glm::vec3>   boundarySet;       // Used for the guard for duplicated points
    std::vector<Hit>                boundaryPoints;    // Selected boundary information
    std::vector<glm::vec2>          boundaryOnPlane;   // Boundary points casted onto plane
    std::vector<glm::vec2>          discretizedPoints; // Discretized points in the boundary
};