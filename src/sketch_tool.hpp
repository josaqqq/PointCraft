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
    void registerSketchPointsAsCurveNetworkLoop(std::string name);
    void removeCurveNetworkLoop(std::string name);

    /*
      Geometry functions
    */

    // Add the specified point to sketchPoints
    void addSketchPoint(glm::dvec3 p);

    // Find basis points.
    //  - If extendedSearch is true, extend the sketched area.
    //  - Cast all points of the point cloud onto the screen plane.
    //  - Check the conditions below.
    //    1. Judge inside/outside of the sketch.
    //    2. Check the normal direction of the point.
    //    3. Check the nearest neighbors' distances from cameraOrig.
    void findBasisPoints(bool extendedSearch);

    // Check whether (x, y) is inside or outside of the sketch.
    //  1.  Draw a half-line parallel to the x-axis from a point.
    //  2.  Determine that if there are an odd number of intersections
    //      between this half-line and the polygon, it is inside, and if 
    //      there are an even number, it is outside.
    bool insideSketch(double x, double y);

    // Check whether (x, y) is inside or outside of the mappedBasisConvexHull.
    //  1.  Draw a half-line parallel to the x-axis from a point.
    //  2.  Determine that if there are an odd number of intersections
    //      between this half-line and the polygon, it is inside, and if 
    //      there are an even number, it is outside.
    bool insideBasisConvexHull(double x, double y);

    // Calculate averageDistance casted onto the screen
    double calcCastedAverageDist();

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

    double      screenDist;    // Distance between screen and camera position
    glm::dvec3  cameraOrig;   // Camera position
    glm::dvec3  cameraDir;    // Camera direction
    Plane       screen;       // Plane on nearClip

    std::vector<glm::dvec3>   sketchPoints;           // Sketched points on the camera screen
    std::vector<int>          basisPointsIndex;       // The indices of selected basis points
    std::vector<glm::dvec2>   mappedBasisConvexHull;  // Convex hull of the basis points mapped onto screen

    // Extend sketched area by averageDistance casted onto the screen.
    void extendSketchedArea();

    // Shrink basisConvexHull to remove the selected points near the boundary.
    void shrinkBasisConvexHull();

    // Calculate CCW value
    //  - ccw > 0.0: left turn
    //  - ccw < 0.0: right turn
    //  - ccw == 0.0: parallel
    double calcCCW(glm::dvec2 p, glm::dvec2 a, glm::dvec2 b);

    // Calculate the convex hull of basisPoints
    void calcBasisConvexHull();

    // Return whether half-line from (x, y) crosses u-v.
    bool crossLines(double x, double y, glm::dvec2 u, glm::dvec2 v);
};