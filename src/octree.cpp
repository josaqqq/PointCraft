#include "octree.hpp"

#include <vector>

Octree::Octree() : octree(128.0f) {}

Octree::Octree(Eigen::MatrixXd vertices, double resolution) : meshV(vertices), octree(resolution) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
  inputCloud->points.resize(vertices.rows());
  for (int i = 0; i < vertices.rows(); i++) {
    inputCloud->points[i].x = vertices(i, 0);
    inputCloud->points[i].y = vertices(i, 1);
    inputCloud->points[i].z = vertices(i, 2);
  }

  octree.setInputCloud(inputCloud);
  octree.addPointsFromInputCloud();
}

// Search nearest neighbor for each point, 
// and then calculate average distance for
// the point cloud. 
double Octree::calcAverageDistance() {
  const int K = 2;
  double averageDistance = 0.0;
  
  for (int i = 0; i < meshV.rows(); i++) {
    std::vector<int> pointIdx;
    std::vector<float> pointDistance;

    pcl::PointXYZ searchPoint(meshV(i, 0), meshV(i, 1), meshV(i, 2));

    int foundNum = octree.nearestKSearch(
      searchPoint,
      K,
      pointIdx,
      pointDistance
    );

    if (foundNum > 1) averageDistance += sqrt(pointDistance[1]);
  }

  return averageDistance / meshV.rows();
}
