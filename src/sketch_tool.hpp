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

    // Run the tool 
    virtual void launchToolOperation() {};

    /*
      Viewer functions
    */
    // Register/Remove point cloud with name.
    // Be aware that the point cloud with 
    // the same name is overwritten.
    void registerSurfacePointsAsPointCloud(std::string name);
    void registerSketchPointsAsPointCloud(std::string name);
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

    // Display voxels for each specified point.
    void displayVoxels(std::vector<glm::dvec3> &points);

    /*
      Geometry functions
    */
    // Compute the surface points where mouse is currently hovering
    // and then update surfacePointsIndex
    //  - xPos: io.DisplayFramebufferScale.x * mousePos.x
    //  - xPos: io.DisplayFramebufferScale.y * mousePos.y
    //  - K_size: the selected nearest neighbors size
    void updateSurfacePoints(double xPos, double yPos, int K_size);

    // Add the specified point to sketchPoints
    void addSketchPoint(glm::dvec3 p);

    // Find basis points.
    //  - Cast points of the point cloud onto the screen plane.
    //  - Construct octree for the casted surface points
    //  - Search for a candidate point for each discretized grid.
    //    - Only points that their normals are directed to cameraOrig
    //  - Detect depth with DBSCAN
    void findBasisPoints();

    // Find all basis points inside the sketch
    //  - Cast points of the point cloud onto the screen plane.
    //  - Judge inside/outside of the sketch.
    //  - Detect depth with DBSCAN.
    void findAllBasisPoints();

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

    // Filter the points on the interpolated surface.
    // Considering points of the surface and the point cloud,
    // select the candidate point of each voxel.
    // Voxel size is averageDistance per side
    //  - filteredPoints: Filtering target
    std::set<int> filterWithVoxel(std::vector<glm::dvec3> &filteredPoints);

    // Calculate averageDistance casted onto the screen
    double calcCastedAverageDist();

    // Reset member variables (vector and set)
    void resetSurfacePointsIndex();
    void resetSketchPoints();
    void resetBasisPointsIndex();
    void resetMappedBasisConvexHull();

    // Return the pointer to member variables.
    PointCloud*               getPointCloud();
    glm::dvec3                getCameraOrig();
    glm::dvec3                getCameraDir();
    Plane*                    getScreen();
    std::set<int>*            getSurfacePointsIndex();
    std::vector<glm::dvec3>*  getSketchPoints();
    std::set<int>*            getBasisPointsIndex();
    std::vector<glm::dvec2>*  getMappedBasisConvexHull();

  private:
    int *currentMode;         // Current selected mode

    PointCloud *pointCloud;   // Registered point cloud

    double      screenDist;   // Distance between screen and camera position
    glm::dvec3  cameraOrig;   // Camera position
    glm::dvec3  cameraDir;    // Camera direction
    Plane       screen;       // Plane on nearClip

    std::set<int>             surfacePointsIndex;     // The indices of surface points where the mouse is hovering
    std::vector<glm::dvec3>   sketchPoints;           // Sketched points on the camera screen
    std::set<int>             basisPointsIndex;       // The indices of selected basis points
    std::vector<glm::dvec2>   mappedBasisConvexHull;  // Convex hull of the basis points mapped onto screen

    // Extend sketched area by averageDistance casted onto the screen.
    void extendSketchedArea();

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