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

    // Register/Remove Patch as point cloud with patchName.
    // Be aware that the point cloud with 
    // the same name is overwritten.
    void registerPatchAsPointCloud(std::string patchName);
    void registerPatchAsPointCloud(Eigen::MatrixXd vertices, Eigen::MatrixXd normals);
    void removePatchAsPointCloud(std::string patchName);

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

    // Return the pointer to member variables.
    double                    getAverageDepth();
    PointCloud*               getPointCloud();
    std::vector<Hit>*         getBoundaryPoints();
    std::vector<glm::dvec2>*  getBoundaryOnPlane();
    std::vector<glm::dvec2>*  getDiscretizedPoints();
    std::pair<glm::dvec3, glm::dvec3> getOrthogonalBasis();

  private:
    PointCloud *pointCloud;

    int *currentMode;     // Current selected Mode
    double averageDepth;  // Average depth of selected points
    // std::unordered_set<glm::dvec3>   boundarySet;       // Used for the guard for duplicated points
    std::vector<Hit>                boundaryPoints;    // Selected boundary information
    std::vector<glm::dvec2>         boundaryOnPlane;   // Boundary points casted onto plane
    std::vector<glm::dvec2>         discretizedPoints; // Discretized points in the boundary

    // Calculate orthogonal basis with Gram-Schmidt orthonormalization.
    glm::dvec3 orthoU, orthoV;  // orthogonal basis for "camera plane"
    void calcOrthogonalBasis(glm::dvec3 normal);

    // Check the inside/outside of the polygon.
    //  1.  Draw a half-line parallel to the x-axis from a point.
    //  2.  Determine that if there are an odd number of intersections
    //      between this half-line and the polygon, it is inside, and if 
    //      there are an even number, it is outside.
    bool insidePolygon(double x, double y, const int polygonSiz);
};