#pragma once

#include <glm/glm.hpp>
#include <unordered_set>

#include "point_cloud.hpp"
#include "plane.hpp"

class Clustering {
  public:
    Clustering(std::vector<int> *pointsIndex, PointCloud *pointCloud, Plane *plane) 
    : pointsIndex(pointsIndex), pointCloud(pointCloud), plane(plane) {}

    // Execute DBSCAN and return selected basis points's index
    std::vector<int> executeDBSCAN(double eps, int minPoints);

  private:
    std::vector<int> *pointsIndex;
    PointCloud       *pointCloud;
    Plane            *plane;
};