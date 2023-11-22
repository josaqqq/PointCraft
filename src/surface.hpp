#pragma once

#include <string>
#include <Eigen/Dense>

// Reconstruct surface with vertex and normal infromation.
void poissonReconstruct(
  std::string name,
  double averageDistance,
  Eigen::MatrixXd vertices,
  Eigen::MatrixXd normals
);

// Smooth vertices with MLS.
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> mlsSmoothing(
  std::string name,
  Eigen::MatrixXd vertices
);

// Triangulate point cloud greedily.
void greedyProjection(
  std::string name,
  Eigen::MatrixXd vertices,
  Eigen::MatrixXd normals
);

// Show hexagons for each vertex as a pseudo surface.
void pseudoSurface(
  std::string name,
  double averageDistance,
  Eigen::MatrixXd vertices,
  Eigen::MatrixXd normals
);