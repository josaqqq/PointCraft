#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"

#include <igl/readOBJ.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <Eigen/Dense>

#include "point_cloud.hpp"
#include "constants.hpp"
#include "surface.hpp"

PointCloud::PointCloud(std::string filename) : octree(OctreeResolution) {
  std::cout << "loading: " << filename << std::endl;

  // Extract file format
  if (filename.length() < 3) exit(1);
  std::string fileFormat = filename.substr(filename.size() - 3);

  Eigen::MatrixXd _V;      // double matrix of vertex positions
  Eigen::MatrixXi _F;      // list of face indices into vertex positions
  Eigen::MatrixXd _N;      // double matrix of corner normals
  Eigen::MatrixXd _TC;     // double matrix of texture coordinates
  Eigen::MatrixXi _FTC;    // list of face indices into vertex texture coordinates
  Eigen::MatrixXi _FN;     // list of face indices into vertex normals

  if (fileFormat == "obj") {
    // Read .obj
    igl::readOBJ(filename, _V, _TC, _N, _F, _FTC, _FN);
    if (_N.rows() == 0) std::cerr << "ERROR: Please include normal information." << std::endl;

    // Initialize Vertices, Normals
    assert(_V.rows() == _N.rows());
    Vertices.resize(_V.rows());
    Normals.resize(_N.rows());
    for (long int i = 0; i < _V.rows(); i++) {
      Vertices[i] = glm::dvec3(_V(i, 0), _V(i, 1), _V(i, 2));
      Normals[i] = glm::dvec3(_N(i, 0), _N(i, 1), _N(i, 2));
    }
  } else if (fileFormat == "pcd") {
    // Read .pcd
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    if (pcl::io::loadPCDFile<pcl::PointNormal>(filename, *cloud) == -1) {
      std::cerr << "ERROR: Couldn't read file" << std::endl;
      exit(1);
    }

    // Initialize Vertices, Normals
    int cloudSize = cloud->points.size();
    Vertices.resize(cloudSize);
    Normals.resize(cloudSize);
    for (int i = 0; i < cloudSize; i++) {
      auto p = cloud->points[i];
      Vertices[i] = glm::dvec3(p.x, p.y, p.z);
      Normals[i] = glm::dvec3(p.normal_x, p.normal_y, p.normal_z);
    }

    // TODO: We need to estimate points' normals here
  }

  // Initialize member variables
  averageDistance = 0.0;
  boundingSphereRadius = 0.0;

  // Update scaling
  scalePointCloud();

  // Update point cloud
  //    - update environments
  //    - update octree
  //    - render points and normals
  updatePointCloud(false);
} 

// Enable or Disable the point cloud and normals
void PointCloud::setPointCloudEnabled(bool flag) {
  pointCloud->setEnabled(flag);
}
void PointCloud::setPointCloudNormalEnabled(bool flag) {
  vectorQuantity->setEnabled(flag);
}

// Update point cloud
//    - update environments
//    - update octree
//    - render points and normals
void PointCloud::updatePointCloud(bool clearPostEnv) {
  // Push new environment
  prevEnvironments.push({ Vertices, Normals });
  if (clearPostEnv) postEnvironments = std::stack<std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>>>();

  // Update Octree
  updateOctree();

  // Calculate average distance between the nearest points.
  if (averageDistance == 0.0) averageDistance = calcAverageDistance();

  // Register Points
  pointCloud = polyscope::registerPointCloud(PointName, Vertices);
  pointCloud->setPointColor(PointColor);
  pointCloud->setPointRadius(PointRadius);
  pointCloud->setEnabled(PointEnabled);

  // Register Normals
  vectorQuantity = pointCloud->addVectorQuantity(NormalName, Normals);
  vectorQuantity->setVectorColor(NormalColor);
  vectorQuantity->setVectorLengthScale(NormalLength);
  vectorQuantity->setVectorRadius(NormalRadius);
  vectorQuantity->setEnabled(NormalEnabled);
  vectorQuantity->setMaterial(NormalMaterial);

  std::cout << "Point Cloud Data:"                                << std::endl;
  std::cout << "\tVertex num\t\t->\t"       << Vertices.size()    << std::endl;
  std::cout << "\tNormal num\t\t->\t"       << Normals.size()     << std::endl;
  std::cout << "\tAverage Distance\t->\t"   << averageDistance    << std::endl;

  // Show Pseudo Surface
  Surface pseudoSurface(PseudoSurfaceName, &Vertices, &Normals);
  pseudoSurface.showPseudoSurface(averageDistance);
}

// Add vertices from the positions and normals
void PointCloud::addPoints(std::vector<glm::dvec3> &newV, std::vector<glm::dvec3> &newN) {
  assert(newV.size() == newN.size());

  int addedSize = newV.size();
  for (int i = 0; i < addedSize; i++) {
    Vertices.push_back(newV[i]);
    Normals.push_back(newN[i]);
  }

  // Update point cloud
  //    - update environments
  //    - update octree
  //    - render points and normals
  updatePointCloud(true);
}

// Delete vertices by referencing the vertex indices
void PointCloud::deletePoints(std::vector<int> &indices) {
  const int curSize = Vertices.size();
  const int newSize = Vertices.size() - indices.size();

  std::vector<glm::dvec3> newV;
  std::vector<glm::dvec3> newN;

  std::vector<int>::iterator itr = indices.begin();
  for (int i = 0; i < curSize; i++) {
    if (itr != indices.end() && i == *itr) {
      itr++;
    } else {
      newV.push_back(Vertices[i]);
      newN.push_back(Normals[i]);
    }
  }

  Vertices = newV;
  Normals = newN;

  // Update point cloud
  //    - update environments
  //    - update octree
  //    - render points and normals
  updatePointCloud(true);
}

// Execute Undo/Redo
void PointCloud::executeUndo() {
  if (prevEnvironments.size() < 2) return;

  // Undo
  std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> currentEnv = prevEnvironments.top();
  prevEnvironments.pop();
  postEnvironments.push(currentEnv);

  std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> newEnv = prevEnvironments.top();
  prevEnvironments.pop();
  Vertices = newEnv.first;
  Normals = newEnv.second;

  // Update point cloud
  //    - update environments
  //    - update octree
  //    - render points and normals
  updatePointCloud(false); 
}
void PointCloud::executeRedo() {
  if (postEnvironments.size() < 1) return;

  // Redo
  std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> newEnv = postEnvironments.top();
  postEnvironments.pop();
  
  Vertices = newEnv.first;
  Normals = newEnv.second;

  // Update point cloud
  //    - update environments
  //    - update octree
  //    - render points and normals
  updatePointCloud(false);
}

// Return the pointer to member variables
std::vector<glm::dvec3>* PointCloud::getVertices() {
  return &Vertices;
}
std::vector<glm::dvec3>* PointCloud::getNormals() {
  return &Normals;
}
double PointCloud::getAverageDistance() {
  return averageDistance;
}
double PointCloud::getBoundingSphereRadius() {
  return boundingSphereRadius;
}
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* PointCloud::getOctree() {
  return &octree;
}

// Move points to set the gravity point to (0.0, 0.0, 0.0),
// and then calculate bounding sphere radius.
void PointCloud::scalePointCloud() {
  glm::dvec3 center;

  for (size_t i = 0; i < Vertices.size(); i++) {
    center += Vertices[i];
  }
  center /= static_cast<double>(Vertices.size());

  boundingSphereRadius = 0.0;
  for (size_t i = 0; i < Vertices.size(); i++) {
    Vertices[i] -= center;

    boundingSphereRadius = std::max(
      boundingSphereRadius, 
      glm::length(Vertices[i])
    );
  }
}

// Update registered vertices
void PointCloud::updateOctree() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
  inputCloud->points.resize(Vertices.size());
  for (size_t i = 0; i < Vertices.size(); i++) {
    inputCloud->points[i].x = Vertices[i].x;
    inputCloud->points[i].y = Vertices[i].y;
    inputCloud->points[i].z = Vertices[i].z;
  }

  octree.deleteTree();
  octree.setInputCloud(inputCloud);
  octree.addPointsFromInputCloud();
}

// Calculate average distance between the nearest points.
double PointCloud::calcAverageDistance() {
  const int K = 2;
  averageDistance = 0.0;
  
  for (size_t i = 0; i < Vertices.size(); i++) {
    glm::dvec3 p = Vertices[i];

    // Search for the nearest neighbor.
    std::vector<int>    hitPointIndices;
    std::vector<float>  hitPointDistances;
    int hitPointCount = octree.nearestKSearch(
      pcl::PointXYZ(p.x, p.y, p.z),
      K,
      hitPointIndices,
      hitPointDistances
    );
    if (hitPointCount > 1) averageDistance += sqrt(hitPointDistances[1]);
  }

  return averageDistance / Vertices.size();
}
