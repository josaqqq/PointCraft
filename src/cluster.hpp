#pragma once

#include <glm/glm.hpp>
#include <unordered_set>

#include "point_cloud.hpp"
#include "plane.hpp"

class Clustering {
  public:
    Clustering(std::vector<int> *pointsIndex, PointCloud *pointCloud) 
    : pointsIndex(pointsIndex), pointCloud(pointCloud) {}

    // Execute clustering
    //  - eps: Clustering search distance
    //  - minPoints: Number of points required to make a point a core point
    std::vector<int> executeClustering(double eps, int minPoints);

  private:
    std::vector<int> *pointsIndex;
    PointCloud       *pointCloud;

    // Calculated three orthogonal bases with PCA
    std::vector<glm::dvec3> orthogonalBases;

    // Calculate three orthogonal bases
    void executePCA();

    // Execute DBSCAN and return selected basis points's index
    std::vector<int> executeDBSCAN(double eps, int minPoints, int basisIndex);

    void visualizeCluster(
      int basisIndex,
      std::vector<int> &pointsIndex,
      std::vector<int> &labesl
    );
};