#pragma once

#include "polyscope/point_cloud.h"

#include <Eigen/Dense>
#include <string>

#include <pcl/octree/octree_search.h>

class PointCloud {
  public:
    PointCloud(std::string filename);
    ~PointCloud() {}

    Eigen::MatrixXd Vertices;    // double matrix of vertex positions
    Eigen::MatrixXd Normals;    // double matrix of corner normals

    // Enable or Disable the point cloud and normals
    void setPointCloudEnabled(bool flag);
    void setPointCloudNormalEnabled(bool flag);

    // Update point cloud
    //   - update octree
    //   - render points and normals
    void updatePointCloud();

    // Add points with information of the position and the normal.
    void addPoints(Eigen::MatrixXd newV, Eigen::MatrixXd newN);

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

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree;

    polyscope::PointCloud *pointCloud;
    polyscope::PointCloudVectorQuantity *vectorQuantity;
    polyscope::PointCloudScalarQuantity *scalarQuantity;
};