#pragma once

// Reconstruct surface with vertex and normal infromation
void poissonReconstruct(Eigen::MatrixXd &vertices, Eigen::MatrixXd &normals);

void mlsReconstruct(Eigen::MatrixXd vertices);