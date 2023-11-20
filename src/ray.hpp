#pragma once

#include "polyscope/polyscope.h"
#include "polyscope/view.h"

#include "point_cloud.hpp"
#include "plane.hpp"

class Hit {
  public:
    Hit() {}

    bool hit;           // hit or not
    double t;           // ray parameter
    double depth;       // distance from "camera plane" to the point
    
    glm::dvec3 pos;     // hit point
    glm::dvec3 normal;  // hit point normal
};

class Ray {
  public:
    Ray(double xScreen, double yScreen);  // Ray from { xScreen, yScreen }
    Ray(glm::dvec3 p, glm::dvec3 q);  // Ray from p to q

    // Calculate the depth from the point 
    // to the plane that is orthogonal to the 
    // frontVec of the camera.
    double calcDepthFromCamera(glm::dvec3 point);

    // Check whether this ray intersect
    // with the sphere specified with 
    // center and radius.
    Hit checkSphere(glm::dvec3 center, double radius);

    // Search for the nearest neighbor points
    // along the specified line.
    // The search range is within the range
    // of the searchRadius.
    Hit searchNeighborPoints(double searchRadius, PointCloud *pointCloud);

    // Cast the point to the specified plane
    // that is parallel to "camera plane".
    Hit castPointToPlane(Plane* plane);

  private:
    glm::dvec3 orig;         // Coordinates of the origin of this ray
    glm::dvec3 rayDir;       // Vector of the direction of this ray
    glm::dvec3 cameraDir;    // Vector of the direction of the camera
};
