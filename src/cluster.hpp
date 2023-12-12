#pragma once

#include <glm/glm.hpp>
#include <unordered_set>

#include "point_cloud.hpp"
#include "plane.hpp"

enum ClusteringMode {
  CLUSTER_MAX_SIZE,
  CLUSTER_MIN_DEPTH,
};

class Clustering {
  public:
    Clustering(
      std::set<int> *pointsIndex, 
      std::vector<glm::dvec3> *points, 
      std::string name
    ) : pointsIndex(pointsIndex), points(points), name(name) {}

    // Execute clustering
    //  - eps: Clustering search distance
    //  - minPoints: Number of points required to make a point a core point
    std::set<int> executeClustering(double eps, size_t minPoints, ClusteringMode mode);

  private:
    ClusteringMode          clusteringMode;

    std::set<int>           *pointsIndex;
    std::vector<glm::dvec3> *points;
    std::string             name;

    // Calculated three orthogonal bases with PCA
    std::vector<glm::dvec3> orthogonalBases;

    // Execute DBSCAN and return selected basis points's index
    std::set<int> executeDBSCAN(double eps, size_t minPoints, int basisIndex);

    void visualizeCluster(
      int basisIndex,
      std::vector<int> &labels
    );
};