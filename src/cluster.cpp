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
std::vector<int> Clustering::executeClustering(double eps, int minPoints) {
  // Calculate three orthogonal bases
  executePCA();

  // Execute DBSCAN for each basis
  std::vector<int> selectedCluster;
  for (int i = 0; i < 3; i++) {
    selectedCluster = executeDBSCAN(eps, minPoints, i);
  }

  return selectedCluster;
}

// Calculate three orthogonal bases
void Clustering::executePCA() {
  // Centering the data
  Eigen::VectorXd mean = pointCloud->Vertices.colwise().mean();
  Eigen::MatrixXd centered = pointCloud->Vertices.rowwise() - mean.transpose();

  // Compute covariance matrix
  Eigen::MatrixXd cov = (centered.adjoint() * centered) / double(pointCloud->Vertices.rows() - 1);

  // Compute eigenvalues and eigenvectors of the cov
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(cov);
  Eigen::VectorXd eigenvalues = eigensolver.eigenvalues();
  Eigen::MatrixXd eigenvectors = eigensolver.eigenvectors();

  // Print solved values
  std::cout << "\nEigenvalues:\n" << eigenvalues << std::endl;
  std::cout << "Eigenvectors:\n" << eigenvectors << std::endl;

  // Register orthogonal
  glm::dvec3 cameraDir = polyscope::view::screenCoordsToWorldRay(
    glm::vec2(polyscope::view::windowWidth/2, polyscope::view::windowHeight/2)
  );
  orthogonalBases.resize(3);
  for (int i = 0; i < 3; i++) { 
    orthogonalBases[i] = glm::dvec3(
      eigenvectors(0, i),
      eigenvectors(1, i),
      eigenvectors(2, i)
    );

    // orthogonalBases must face the camera direction
    if (glm::dot(cameraDir, orthogonalBases[i]) < 0.0) orthogonalBases[i] *= -1.0;
  }
}

// Execute DBSCAN and return selected basis points
// implemented referencing https://en.wikipedia.org/wiki/DBSCAN
std::vector<int> Clustering::executeDBSCAN(double eps, int minPoints, int basisIndex) {
  // Initialize points information
  int pointSize = pointsIndex->size();
  std::vector<double> depths(pointSize);

  for (int i = 0; i < pointSize; i++) {
    int idx = (*pointsIndex)[i];

    glm::dvec3 p = glm::dvec3(
      pointCloud->Vertices(idx, 0),
      pointCloud->Vertices(idx, 1),
      pointCloud->Vertices(idx, 2)
    );
    depths[i] = glm::dot(orthogonalBases[basisIndex], p);
  }

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

  std::map<int, int> labelToCount;
  std::map<int, double> labelToDepth;
  for (int i = 0; i < pointSize; i++) {
    labelToCount[labels[i]] += 1;
    labelToDepth[labels[i]] += depths[i];
  }

  int minDepthLabel = -1;
  double minDepth = 1e5;
  for (std::pair<int, double> i: labelToDepth) {
    double averageDepth = i.second / labelToCount[i.first];
    if (std::abs(averageDepth) < minDepth) {
      minDepthLabel = i.first;
      minDepth = averageDepth;
    }
  }

  std::vector<int> basisPointsIndex;
  for (int i = 0; i < pointSize; i++) {
    if (labels[i] == minDepthLabel) basisPointsIndex.push_back((*pointsIndex)[i]);
  }

  // Visualize the depth of points with labels.
  visualizeCluster(basisIndex, *pointsIndex, labels);

  return basisPointsIndex;
}

void Clustering::visualizeCluster(
  int basisIndex,
  std::vector<int> &pointsIndex,
  std::vector<int> &labels
) {
  // Remove previous point cloud.
  polyscope::removePointCloud(DBSCAN_Name);

  int pointSize = pointsIndex.size();

  // Define label map
  std::map<int, std::vector<int>> labelToIndices;
  std::map<int, glm::dvec3>       labelToColors;
  for (int i = 0; i < pointSize; i++) {
    int label = labels[i];

    if (labelToIndices.count(label) == 0) {
      labelToColors[label] = glm::dvec3(
        polyscope::randomUnit(),
        polyscope::randomUnit(),
        polyscope::randomUnit()
      );
    }
    labelToIndices[label].push_back(pointsIndex[i]);
  }

  // Register point and color information
  std::vector<std::vector<double>> labelPoints;
  std::vector<std::vector<double>> labelColors;
  for (auto i: labelToIndices) {
    int label = i.first;

    for (int idx: i.second) {
      // point and point casted on camera line.
      glm::dvec3 p = glm::dvec3(
        pointCloud->Vertices(idx, 0),
        pointCloud->Vertices(idx, 1),
        pointCloud->Vertices(idx, 2)
      );
      glm::dvec3 basis = orthogonalBases[basisIndex];
      glm::dvec3 castedP = glm::dot(basis, p)*basis;

      labelPoints.push_back({ p.x, p.y, p.z });
      labelPoints.push_back({ castedP.x, castedP.y, castedP.z });
      
      glm::dvec3 col = labelToColors[label];
      labelColors.push_back({ col.x, col.y, col.z });
      labelColors.push_back({ col.x, col.y, col.z });
    }
  }

  // Register point cloud
  polyscope::PointCloud* depthCloud = polyscope::registerPointCloud(DBSCAN_Name + std::to_string(basisIndex), labelPoints);
  depthCloud->setPointRadius(PointRadius);
  depthCloud->setEnabled(DBSCAN_Enabled);
  
  polyscope::PointCloudColorQuantity* depthColor = depthCloud->addColorQuantity("label color", labelColors);
  depthColor->setEnabled(true);
}