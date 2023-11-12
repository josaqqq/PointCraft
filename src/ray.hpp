#pragma once

#include "polyscope/polyscope.h"
#include "polyscope/view.h"

class Hit {
  public:
    Hit(glm::vec2 screenCoord) : screenCoord(screenCoord) {}

    bool hit;
    double t;
    
    glm::vec3 pos;
    glm::vec3 normal;

    glm::vec2 screenCoord;
};

class Ray {
  public:
    Ray(double xScreen, double yScreen);

    // Check whether this ray intersect
    // with the sphere specified with 
    // center and radius.
    Hit checkSphere(glm::vec3 center, double radius);

    // Search for the nearest neighbor point
    // along the specified line.
    // The search range is within the range
    // of the searchRadius.
    Hit searchNeighborPoints(double searchRadius);

  private:
    // Coordinates on the screen
    glm::vec2 screenCoord;
    // Coordinates of the origin of this ray
    glm::vec3 orig;
    // Vector of the direction of this ray
    glm::vec3 dir;
};
