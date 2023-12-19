#pragma once

#include "polyscope/point_cloud.h"
#include <pcl/octree/octree_search.h>
#include <chrono>

#include <string>
#include <stack>

class PointCloud {
  public:
    PointCloud(std::string filename, bool downsample);
    ~PointCloud() {}

    // Output current Vertices and Normals as .obj file
    void exportOBJFile();

    // Output log to logFile
    void exportLog(
      std::chrono::high_resolution_clock::time_point start_clock,
      std::string logFileName
    );

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
    int getBoundaryPointNum();
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
    double averageDistance = 0.0;   // Average Distance between a point and the nearest neighbor
    double boundingBoxSide = 0.0;   // Side of bounding sphere of point cloud
    int boundaryPointNum = -1;      // Number of the hole boundary points on the mesh reconstructed from the current point cloud.

    // std::vector<glm::dvec3> as buffer
    //  There is a lag between when the added points are
    //  added to Vertices and Normals, and when they are
    //  actually rendered. VerticesBuffer and NormalsBuffer
    //  are used to hold data until the added points are 
    //  actually rendered.
    std::vector<glm::dvec3> VerticesBuffer;
    std::vector<glm::dvec3> NormalsBuffer;

    // Undo/Redo stacks
    std::stack<std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>>> prevEnvironments;
    std::stack<std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>>> postEnvironments;
    std::vector<std::chrono::high_resolution_clock::time_point> undoTimestamps; // Timestamps of each undo.
    std::vector<std::chrono::high_resolution_clock::time_point> redoTimestamps; // Timestamps of each redo.

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree;

    polyscope::PointCloud *pointCloud;
    polyscope::PointCloudVectorQuantity *vectorQuantity;
    polyscope::PointCloudScalarQuantity *scalarQuantity;
};