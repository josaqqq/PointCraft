#pragma once

#include <glm/glm.hpp>

class Plane {
  public:
    Plane() {}
    Plane(glm::dvec3 o, glm::dvec3 n);

    // Map the point from {x, y, z} to {n, u, v}
    glm::dvec3 mapCoordinates(glm::dvec3 p);

    // Map the point from {n, u, v} to {x, y, z}
    glm::dvec3 unmapCoordinates(glm::dvec3 p);

    // Get private member variables
    glm::dvec3 getOrigin();
    glm::dvec3 getNormal();

  private:
    glm::dvec3 o;      // origin of this coordinate
    glm::dvec3 n;      // normal
    glm::dvec3 u, v;   // basis for this plane
};
