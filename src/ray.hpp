#pragma once

#include "polyscope/polyscope.h"
#include "polyscope/view.h"

#include "point_cloud.hpp"
#include "plane.hpp"

class Hit {
  public:
    Hit() {}

    bool hit;           // hit or not
    
    int index;          // point index
    glm::dvec3 pos;     // hit point
    glm::dvec3 normal;  // hit point normal
};

class Ray {
  public:
    Ray(double xScreen, double yScreen);  // Ray from { xScreen, yScreen }
    Ray(glm::dvec3 p, glm::dvec3 q);  // Ray from p to q

    // Cast the point to the specified plane
    // that is parallel to "camera plane".
    Hit castPointToPlane(Plane* plane);

  private:
    glm::dvec3 rayDir;       // Vector of the direction of this ray
    glm::dvec3 cameraOrig;   // Coordinates of the origin of this ray
    glm::dvec3 cameraDir;    // Vector of the direction of the camera
};
