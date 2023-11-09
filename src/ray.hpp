#pragma once

#include "polyscope/polyscope.h"
#include "polyscope/view.h"

class Hit {
  public:
    bool hit;
    double t;
    glm::vec3 pos;
    glm::vec3 normal;
};

class Ray {
  public:
    Ray(double xScreen, double yScreen);

    // Check whether this ray intersect
    // with the sphere specified with 
    // center and radius.
    Hit checkSphere(glm::vec3 center, double radius);

  private:
    // Coordinates of the origin of this ray
    glm::vec3 orig;
    // Vector of the direction of this ray
    glm::vec3 dir;
};
