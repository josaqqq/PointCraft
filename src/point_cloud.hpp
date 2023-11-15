#pragma once

#include <Eigen/Dense>

#include <string>

#include "octree.hpp"

class PointCloud {
  public:
    PointCloud() {}
    PointCloud(std::string filename);
    ~PointCloud() {}

    Eigen::MatrixXd meshV;    // double matrix of vertex positions
    Eigen::MatrixXd meshN;    // double matrix of corner normals
    Eigen::MatrixXi meshF;    // list of face indices into vertex positions

    double averageDistance;

    // Move points to set the gravity point to (0.0, 0.0, 0.0).
    void movePointsToOrigin();

    // Update point cloud
    //   - update octree
    //   - render points and normals
    void updatePointCloud();

    // Add points with information of the position and the normal.
    void addPoints(Eigen::MatrixXd newV, Eigen::MatrixXd newN);

  private:
    Octree octree;
};