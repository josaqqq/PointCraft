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

  if (fileFormat == "obj") {
    // Read .obj
    Eigen::MatrixXi _F;  // list of face indices into vertex positions
    Eigen::MatrixXd _TC;     // double matrix of texture coordinates
    Eigen::MatrixXi _FTC;    // list of face indices into vertex texture coordinates
    Eigen::MatrixXi _FN;     // list of face indices into vertex normals
    igl::readOBJ(filename, Vertices, _TC, Normals, _F, _FTC, _FN);
    if (Normals.rows() == 0) std::cerr << "ERROR: Please include normal information." << std::endl;
  } else if (fileFormat == "pcd") {
    // Read .pcd
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    if (pcl::io::loadPCDFile<pcl::PointNormal>(filename, *cloud) == -1) {
      std::cerr << "ERROR: Couldn't read file" << std::endl;
      exit(1);
    }

    // Extract vertices and normals
    int cloudSize = cloud->points.size();
    Vertices = Eigen::MatrixXd(cloudSize, 3);
    Normals = Eigen::MatrixXd(cloudSize, 3);
    for (int i = 0; i < cloudSize; i++) {
      Vertices.row(i) << 
        cloud->points[i].x,
        cloud->points[i].y,
        cloud->points[i].z;
      Normals.row(i) << 
        cloud->points[i].normal_x,
        cloud->points[i].normal_y,
        cloud->points[i].normal_z;
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
  if (clearPostEnv) postEnvironments = std::stack<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>>();

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
  std::cout << "\tVertex num\t\t->\t"       << Vertices.rows()    << std::endl;
  std::cout << "\tNormal num\t\t->\t"       << Normals.rows()     << std::endl;
  std::cout << "\tAverage Distance\t->\t"   << averageDistance    << std::endl;

  // Show Pseudo Surface
  Surface pseudoSurface(PseudoSurfaceName, &Vertices, &Normals);
  pseudoSurface.showPseudoSurface(averageDistance);
}

// Add vertices from the positions and normals
void PointCloud::addPoints(Eigen::MatrixXd newV, Eigen::MatrixXd newN) {
  Eigen::MatrixXd concatV = Eigen::MatrixXd::Zero(Vertices.rows() + newV.rows(), Vertices.cols());
  concatV << Vertices, newV;
  Vertices = concatV;

  Eigen::MatrixXd concatN = Eigen::MatrixXd::Zero(Normals.rows() + newN.rows(), Normals.cols());
  concatN << Normals, newN;
  Normals = concatN;

  // Update point cloud
  //    - update environments
  //    - update octree
  //    - render points and normals
  updatePointCloud(true);
}

// Delete vertices by referencing the vertex indices
void PointCloud::deletePoints(std::vector<int> &indices) {
  // Sort indices for the comparison with current vertices information
  sort(indices.begin(), indices.end());

  const int curSize = Vertices.rows();
  const int newSize = Vertices.rows() - indices.size();

  Eigen::MatrixXd newV(newSize, 3);
  Eigen::MatrixXd newN(newSize, 3);

  int curIdx = 0;
  std::vector<int>::iterator itr = indices.begin();
  for (int i = 0; i < curSize; i++) {
    if (itr != indices.end() && i == *itr) {
      itr++;
    } else {
      newV.row(curIdx) << 
        Vertices(i, 0),
        Vertices(i, 1),
        Vertices(i, 2);
      newN.row(curIdx) << 
        Normals(i, 0),
        Normals(i, 1),
        Normals(i, 2);
      curIdx++;
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
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> currentEnv = prevEnvironments.top();
  prevEnvironments.pop();
  postEnvironments.push(currentEnv);

  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> newEnv = prevEnvironments.top();
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
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> newEnv = postEnvironments.top();
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
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  for (int i = 0; i < Vertices.rows(); i++) {
    x += Vertices(i, 0);
    y += Vertices(i, 1);
    z += Vertices(i, 2);
  }
  x /= static_cast<double>(Vertices.rows());
  y /= static_cast<double>(Vertices.rows());
  z /= static_cast<double>(Vertices.rows());

  boundingSphereRadius = 0.0;
  for (int i = 0; i < Vertices.rows(); i++) {
    Vertices(i, 0) -= x;
    Vertices(i, 1) -= y;
    Vertices(i, 2) -= z;

    boundingSphereRadius = std::max(
      boundingSphereRadius, 
      glm::length(glm::dvec3(Vertices(i, 0), Vertices(i, 1), Vertices(i, 2)))
    );
  }
}

// Update registered vertices
void PointCloud::updateOctree() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
  inputCloud->points.resize(Vertices.rows());
  for (int i = 0; i < Vertices.rows(); i++) {
    inputCloud->points[i].x = Vertices(i, 0);
    inputCloud->points[i].y = Vertices(i, 1);
    inputCloud->points[i].z = Vertices(i, 2);
  }

  octree.deleteTree();
  octree.setInputCloud(inputCloud);
  octree.addPointsFromInputCloud();
}

// Calculate average distance between the nearest points.
double PointCloud::calcAverageDistance() {
  const int K = 2;
  averageDistance = 0.0;
  
  for (int i = 0; i < Vertices.rows(); i++) {
    // Search for the nearest neighbor.
    std::vector<int>    hitPointIndices;
    std::vector<float>  hitPointDistances;
    int hitPointCount = octree.nearestKSearch(
      pcl::PointXYZ(Vertices(i, 0), Vertices(i, 1), Vertices(i, 2)),
      K,
      hitPointIndices,
      hitPointDistances
    );
    if (hitPointCount > 1) averageDistance += sqrt(hitPointDistances[1]);
  }

  return averageDistance / Vertices.rows();
}
