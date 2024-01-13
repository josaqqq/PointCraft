#include "polyscope/polyscope.h"

#include "polyscope/point_cloud.h"
#include "polyscope/view.h"

#include <iostream>
#include <queue>
#include <map>

#include <glm/glm.hpp>
#include <glm/gtc/random.hpp>

#include "cluster.hpp"
#include "constants.hpp"

// Execute clustering
//  - eps: Clustering search distance
//  - minPoints: Number of points required to make a point a core point
std::set<int> Clustering::executeClustering(double eps, size_t minPoints, ClusteringMode mode) {
  // Set ClusteringMode
  clusteringMode = mode;

  // Set camera direction
  cameraDir = polyscope::view::screenCoordsToWorldRay(
    glm::vec2(polyscope::view::windowWidth/2, polyscope::view::windowHeight/2)
  );

  // Execute DBSCAN and return hit indices
  return executeDBSCAN(eps, minPoints);
}

// Execute DBSCAN and return selected basis points
// implemented referencing https://en.wikipedia.org/wiki/DBSCAN
std::set<int> Clustering::executeDBSCAN(double eps, size_t minPoints) {
  const int pointSize = pointsIndex->size();

  // Compute the depth of each point
  std::vector<double> depths;
  for (int idx: *pointsIndex) {
    glm::dvec3 p = (*points)[idx];
    depths.push_back(glm::dot(cameraDir, p));
  }

  // Execute DBSCAN
  std::vector<int> labels(pointSize, -1);
  int label = 0;
  for (int i = 0; i < pointSize; i++) {
    if (labels[i] != -1) continue;  // If not undefined, then skip it.

    std::queue<int> neighbors;
    std::unordered_set<int> neighborsAdded;
    // Searh for neighbor points.
    for (int j = 0; j < pointSize; j++) {
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
        for (int q = 0; q < pointSize; q++) {
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

  // Aggregate DBSCAN result
  std::map<int, int> labelToCount;
  std::map<int, double> labelToDepth;
  for (int i = 0; i < pointSize; i++) {
    labelToCount[labels[i]] += 1;
    labelToDepth[labels[i]] += depths[i];
  }

  // Determine the label to be selected
  int selectedLabel = -1;
  int maxSize = 0;
  double minDepth = 1e5;
  double averageDepth;
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

  // Compute return value
  std::set<int> basisPointsIndex;
  auto labelsItr = labels.begin();
  auto pointsIndexItr = pointsIndex->begin();
  while (labelsItr != labels.end() && pointsIndexItr != pointsIndex->end()) {
    int label = *labelsItr;
    int idx = *pointsIndexItr;
    if (label == selectedLabel) basisPointsIndex.insert(idx);
    
    labelsItr++;
    pointsIndexItr++;
  }

  // Visualize the depth of points with labels.
  visualizeCluster(labels);

  return basisPointsIndex;
}

void Clustering::visualizeCluster(std::vector<int> &labels) {
  // Remove previous point cloud.
  polyscope::removePointCloud(DBSCAN_Name);

  // Compute mapping from label to index and color
  std::map<int, std::vector<int>> labelToIndices;
  std::map<int, glm::dvec3>       labelToColors;

  auto labelsItr = labels.begin();
  auto pointsIndexItr = pointsIndex->begin();
  while (labelsItr != labels.end() && pointsIndexItr != pointsIndex->end()) {
    int label = *labelsItr;
    int idx = *pointsIndexItr;
    if (labelToIndices.count(label) == 0) {
      labelToColors[label] = glm::dvec3(
        polyscope::randomUnit(),
        polyscope::randomUnit(),
        polyscope::randomUnit()
      );
    }
    labelToIndices[label].push_back(idx);
  
    labelsItr++;
    pointsIndexItr++;
  }

  // Register point and color information
  std::vector<glm::dvec3> labeledPoints;
  std::vector<glm::dvec3> castedPoints;
  std::vector<glm::dvec3> labeledColor;
  for (auto i: labelToIndices) {
    int label = i.first;

    for (int idx: i.second) {
      // point and point casted on camera line.
      glm::dvec3 p = (*points)[idx];
      glm::dvec3 casted_p = glm::dot(cameraDir, p)*cameraDir;

      labeledPoints.push_back(p);
      castedPoints.push_back(casted_p);

      glm::dvec3 col = labelToColors[label];
      labeledColor.push_back(col);
    }
  }

  // Register labeled points in 3D coordinate
  polyscope::PointCloud* labeledCloud = polyscope::registerPointCloud(DBSCAN_Name+"::"+name+"::labeled points", labeledPoints);
  labeledCloud->setPointRadius(PointRadius);
  labeledCloud->setPointColor(BasisPointColor);
  labeledCloud->setEnabled(DBSCAN_Enabled);
  polyscope::PointCloudColorQuantity* labeledColorPtr = labeledCloud->addColorQuantity("label color", labeledColor);
  labeledColorPtr->setEnabled(true);

  // Register casted points onto the camera ray
  polyscope::PointCloud* castedCloud = polyscope::registerPointCloud(DBSCAN_Name+"::"+name+"::casted points", castedPoints);
  castedCloud->setPointRadius(PointRadius);
  castedCloud->setPointColor(BasisPointColor);
  castedCloud->setEnabled(DBSCAN_Enabled);
  polyscope::PointCloudColorQuantity* castedColorPtr = castedCloud->addColorQuantity("label color", labeledColor);
  castedColorPtr->setEnabled(true);
}