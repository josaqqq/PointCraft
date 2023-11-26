#pragma once

#include "polyscope/polyscope.h"

#include "ray.hpp"
#include "plane.hpp"
#include "point_cloud.hpp"

class SketchTool {
  public:
    SketchTool(int *currentMode, PointCloud *pointCloud) 
    : currentMode(currentMode), pointCloud(pointCloud) {}
    virtual ~SketchTool() {}

    /*
      Manage functions
    */

    // Initialize member variables.
    void initSketch();

    // Reset all member variables.
    void resetSketch();

    // Draw sketch according to the selected mode.
    virtual bool drawSketch() { return false; };

    /*
      Viewer functions
    */

    // Register/Remove point cloud with name.
    // Be aware that the point cloud with 
    // the same name is overwritten.
    void registerBasisPointsAsPointCloud(std::string name);
    void removePointCloud(std::string name);

    // Register/Remove curve network line with name.
    // Be aware that the curve network with 
    // the same name is overwritten
    void registerSketchPointsAsCurveNetworkLine(std::string name);
    void removeCurveNetworkLine(std::string name);

    // Register/Remove curve network loop with name.
    // Be aware that the curve network with 
    // the same name is overwritten
    void registerSketchAsCurveNetworkLoop(std::string name);
    void removeCurveNetworkLoop(std::string name);

    /*
      Geometry functions
    */

    // Add the specified point to sketchPoints
    void addSketchPoint(glm::dvec3 p);

    // Find basis points.
    //  - Cast all points of the point cloud onto the screen plane.
    //  - Judge inside/outside of the sketch.
    //  - Check whether the normal and the camera direction are faced each other.
    void findBasisPoints();

    // Check the inside/outside of the polygon.
    //  1.  Draw a half-line parallel to the x-axis from a point.
    //  2.  Determine that if there are an odd number of intersections
    //      between this half-line and the polygon, it is inside, and if 
    //      there are an even number, it is outside.
    bool insideSketch(double x, double y);

    // Return the pointer to member variables.
    PointCloud*               getPointCloud();
    double                    getAverageDepth();
    glm::dvec3                getCameraOrig();
    glm::dvec3                getCameraDir();
    Plane*                    getScreen();
    std::vector<glm::dvec3>*  getSketchPoints();
    std::vector<int>*         getBasisPointsIndex();

  private:
    int *currentMode;         // Current selected Mode

    PointCloud *pointCloud;   // Registered point cloud
    double      averageDepth; // Average depth of selected points

    glm::dvec3  cameraOrig;   // Camera position
    glm::dvec3  cameraDir;    // Camera direction
    Plane       screen;       // Plane on nearClip

    std::vector<glm::dvec3>   sketchPoints;     // Sketched points on the camera screen
    std::vector<int>          basisPointsIndex;  // The indices of selected basis points
};