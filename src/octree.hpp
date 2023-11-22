#pragma once

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

#include <Eigen/Dense>

class Octree {
  public:
    Octree();
    Octree(Eigen::MatrixXd meshV, double resolution);
    ~Octree() {}

    // Search the nearest neighbor for each point, 
    // and then calculate average distance for
    // the point cloud.
    double calcAverageDistance();

  private:
    Eigen::MatrixXd meshV;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree;
};