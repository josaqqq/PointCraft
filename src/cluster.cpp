#include <iostream>
#include <queue>
#include <map>
#include <set>

#include "cluster.hpp"
#include "point_cloud.hpp"

// Execute clustering
//  - eps: Clustering search distance
//  - minPoints: Number of points required to make a point a core point
std::vector<Vertex> Clustering::executeClustering(double eps, size_t minPoints) {
	const int verticesSize = vertices->size();
	
	/* Compute the depth of each point */
	std::vector<double> depths;
  for (Vertex v: *vertices) {
    glm::vec3 p = glm::vec3(v.x, v.y, v.z);
    depths.emplace_back(glm::dot(cameraDir, p));
  }
	
	/* Execute DBSCAN */
  std::vector<int> labels(verticesSize, -1);
  int label = 0;
  for (int i = 0; i < verticesSize; i++) {
    if (labels[i] != -1) continue;  // If not undefined, then skip it.

    std::queue<int> neighbors;
    std::set<int> neighborsAdded;
    // Searh for neighbor points.
    for (int j = 0; j < verticesSize; j++) {
      if (std::abs(depths[i] - depths[j]) <= eps) {
        neighbors.push(j);
        neighborsAdded.insert(j);
      }
    }

    if (neighbors.size() < minPoints) {
      // If the number of neighbors is smaller than the threshold, 
      // then it is judged as noise.
      labels[i] = 0;
    } else {
      label++;
      labels[i] = label;  // point-i is labled

      // Extend the clusters while neighbors contains points.
      int neighborsCount = 0;
      while (neighbors.size()) {
        neighborsCount++;
        int p = neighbors.front(); neighbors.pop();

        if (labels[p] == 0) labels[p] = label;  // Change noise to border point
        if (labels[p] != -1) continue;          // Previously processed (e.g., border point)

        labels[p] = label;  // point-p is labled
        // Search for new neighbor points.
        std::vector<int> newNeighbors;
        for (int q = 0; q < verticesSize; q++) {
          if (std::abs(depths[p] - depths[q]) <= eps) newNeighbors.push_back(q);
        }

        // If point-p is a core point, then update neighbors.
        if (newNeighbors.size() >= minPoints) {
          for (int q: newNeighbors) {
            if (neighborsAdded.count(q)) continue;
            neighbors.push(q);
            neighborsAdded.insert(q);
          }
        }
      }
    }
  }

  /* Aggregate DBSCAN result */
  std::map<int, int> labelToCount;
  std::map<int, double> labelToDepth;
  for (int i = 0; i < verticesSize; i++) {
    labelToCount[labels[i]] += 1;
    labelToDepth[labels[i]] += depths[i];
  }

  /* Determine the label to be selected */
  int selectedLabel = -1;
  int maxSize = 0;
  double minDepth = 1e5;
  switch (clusteringMode) {
    case CLUSTER_MAX_SIZE:
      for (std::pair<int, int> i: labelToCount) {
        if (i.second > maxSize) {
          selectedLabel = i.first;
          maxSize = i.second;
        }
      }
      break;
    case CLUSTER_MIN_DEPTH:
      for (std::pair<int, double> i: labelToDepth) {
        double averageDepth = i.second / labelToCount[i.first];
        if (averageDepth < minDepth) {
          selectedLabel = i.first;
          minDepth = averageDepth;
        }
      }
      break;
  }

  /* Compute return value */
  std::vector<Vertex> basisPoints;
  for (int i = 0; i < verticesSize; i++) {
    if (labels[i] == selectedLabel) {
      basisPoints.emplace_back((*vertices)[i]);
    }
  }

  // Visualize the depth of points with labels.
  visualizeCluster(labels);

  return basisPoints;
}

void Clustering::visualizeCluster(std::vector<int>& labels) {

}
