#pragma once

#include <glm/glm.hpp>

#include "point_cloud.hpp"

enum ClusteringMode {
  CLUSTER_MAX_SIZE,
  CLUSTER_MIN_DEPTH,
};

class Clustering {
  public:
    Clustering(
      std::vector<Vertex>* vertices,
      glm::vec3 cameraDir,
      int clusteringMode
    ) : vertices(vertices), cameraDir(cameraDir), clusteringMode(clusteringMode) {}

    // Execute clustering
    //  - eps: Clustering search distance
    //  - minPoints: Number of points required to make a point a core point
    std::vector<Vertex> executeClustering(double eps, size_t minPoints);

  private:
		// Visualize the result of clustering
    void visualizeCluster(std::vector<int>& labels);

    std::vector<Vertex>* vertices;
    glm::vec3 cameraDir;
    int clusteringMode;

    // Constant member variables
    const double DBSCAN_SearchRange = 1.0;
    const int DBSCAN_MinPoints = 1;
};