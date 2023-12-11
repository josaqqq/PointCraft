#pragma once

#include "polyscope/point_cloud.h"
#include <pcl/octree/octree_search.h>

#include <string>
#include <stack>

class PointCloud {
  public:
    PointCloud(std::string filename, bool downsample);
    ~PointCloud() {}

    // Enable or Disable the point cloud and normals
    void setPointCloudEnabled(bool flag);
    void setPointCloudNormalEnabled(bool flag);

    // Update point cloud
    //    - update environments
    //    - update octree
    //    - render points and normals
    void updatePointCloud(bool clearPostEnv);

    // Add vertices from the positions and normals
    void addPoints(std::vector<glm::dvec3> &newV, std::vector<glm::dvec3> &newN);

    // Delete vertices by referencing the vertex indices
    void deletePoints(std::set<int> &indices);

    // Execute Undo/Redo
    void executeUndo();
    void executeRedo();

    // Return the pointer to member variables
    std::vector<glm::dvec3>* getVertices();
    std::vector<glm::dvec3>* getNormals();
    double getAverageDistance();
    double getBoundingBoxSide();
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* getOctree();

  private:
    // Move points to set the gravity point to (0.0, 0.0, 0.0),
    // and then scale the point cloud so that the bounding box side is 1.0
    void scalePointCloud();

    // Filter Vertices by selecting the candidate
    // point for each voxel.
    //  - voxelSide:      the length of the voxel side
    std::set<int> downsampling(double voxelSide);

    // Update registered vertices
    void updateOctree();

    // Calculate average distance between the nearest points.
    double calcAverageDistance();

    // Point cloud information
    std::vector<glm::dvec3> Vertices;
    std::vector<glm::dvec3> Normals;
    double averageDistance;       // Average Distance between a point and the nearest neighbor
    double boundingBoxSide;  // Side of bounding sphere of point cloud

    // Undo/Redo stacks
    std::stack<std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>>> prevEnvironments;
    std::stack<std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>>> postEnvironments;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree;

    polyscope::PointCloud *pointCloud;
    polyscope::PointCloudVectorQuantity *vectorQuantity;
    polyscope::PointCloudScalarQuantity *scalarQuantity;
};