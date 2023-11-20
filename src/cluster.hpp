#pragma once

#include <glm/glm.hpp>
#include <unordered_set>

#include "plane.hpp"

class Clustering {
  public:
    Clustering(std::unordered_set<glm::dvec3> *points, Plane *plane) 
    : pointsSet(points), plane(plane) {}

    // Execute DBSCAN and return selected basis points
    std::vector<glm::dvec3> executeDBSCAN(double eps, int minPoints);

  private:
    std::unordered_set<glm::dvec3> *pointsSet;
    Plane *plane;
};