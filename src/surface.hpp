#pragma once

#include <Eigen/Dense>

// Reconstruct surface with vertex and normal infromation.
void poissonReconstruct(Eigen::MatrixXd vertices, Eigen::MatrixXd normals);

// Smooth vertices with MLS.
void mlsSmoothing(Eigen::MatrixXd vertices);

// Triangulate point cloud greedily.
void greedyProjection(Eigen::MatrixXd vertices, Eigen::MatrixXd normals);