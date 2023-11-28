#pragma once

#include "polyscope/point_cloud.h"
#include <pcl/octree/octree_search.h>
#include <Eigen/Dense>

#include <string>
#include <stack>

class PointCloud {
  public:
    PointCloud(std::string filename);
    ~PointCloud() {}

    Eigen::MatrixXd Vertices;   // double matrix of vertex positions
    Eigen::MatrixXd Normals;    // double matrix of corner normals

    // Enable or Disable the point cloud and normals
    void setPointCloudEnabled(bool flag);
    void setPointCloudNormalEnabled(bool flag);

    // Update point cloud
    //    - update environments
    //    - update octree
    //    - render points and normals
    void updatePointCloud(bool clearPostEnv);

    // Add vertices from the positions and normals
    void addPoints(Eigen::MatrixXd newV, Eigen::MatrixXd newN);

    // Delete vertices by referencing the vertex indices
    void deletePoints(std::vector<int> &indices);

    // Execute Undo/Redo
    void executeUndo();
    void executeRedo();

    // Return the pointer to member variables
    double getAverageDistance();
    double getBoundingSphereRadius();
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* getOctree();

  private:
    // Move points to set the gravity point to (0.0, 0.0, 0.0),
    // and then calculate bounding sphere radius.
    void scalePointCloud();

    // Update registered vertices
    void updateOctree();

    // Calculate average distance between the nearest points.
    double calcAverageDistance();

    double averageDistance;       // Average Distance between a point and the nearest neighbor
    double boundingSphereRadius;  // Radius of bounding sphere of point cloud

    std::stack<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> prevEnvironments;
    std::stack<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> postEnvironments;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree;

    polyscope::PointCloud *pointCloud;
    polyscope::PointCloudVectorQuantity *vectorQuantity;
    polyscope::PointCloudScalarQuantity *scalarQuantity;
};