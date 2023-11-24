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

  // Update scaling
  scalePointCloud();

  // Update point cloud
  //   - update octree
  //   - render points and normals
  updatePointCloud();
} 

// Enable or Disable the point cloud and normals
void PointCloud::setPointCloudEnabled(bool flag) {
  pointCloud->setEnabled(flag);
}
void PointCloud::setPointCloudNormalEnabled(bool flag) {
  vectorQuantity->setEnabled(flag);
}

// Update point cloud
//   - update octree
//   - render points and normals
void PointCloud::updatePointCloud() {
  // Update Octree
  updateOctree();

  // Calculate average distance between the nearest points.
  averageDistance = calcAverageDistance();

  // Register Points
  pointCloud = polyscope::registerPointCloud(PointName, Vertices);
  pointCloud->setPointColor(PointColor);
  pointCloud->setPointRadius(PointRadius);

  // Register Normals
  vectorQuantity = pointCloud->addVectorQuantity(NormalName, Normals);
  vectorQuantity->setVectorColor(NormalColor);
  vectorQuantity->setVectorLengthScale(NormalLength);
  vectorQuantity->setVectorRadius(NormalRadius);
  vectorQuantity->setEnabled(NormalEnabled);
  vectorQuantity->setMaterial(NormalMaterial);

  // // Register Scalar Quantity
  // std::vector<double> scalarValues(Vertices.rows());
  // for (int i = 0; i < Vertices.rows(); i++) scalarValues[i] = Vertices(i, 2);
  // scalarQuantity = pointCloud->addScalarQuantity(ScalarName, scalarValues);
  // scalarQuantity->setColorMap(ScalarColorMap);
  // scalarQuantity->setEnabled(ScalarEnabled);

  std::cout << "Point Cloud Data:"                               << std::endl;
  std::cout << "\tVertex num:\t\t"            << Vertices.rows()  << std::endl;
  std::cout << "\tNormal num:\t\t"            << Normals.rows()   << std::endl;
  std::cout << "\tAverage Distance\t"         << averageDistance  << std::endl;

  // Reconstruct Surfaces
  poissonReconstruct(PoissonName, averageDistance, Vertices, Normals);
  greedyProjection(GreedyProjName, Vertices, Normals);
  pseudoSurface(PseudoSurfaceName, averageDistance, Vertices, Normals);
}

// Add points with information of the position and the normal.
void PointCloud::addPoints(Eigen::MatrixXd newV, Eigen::MatrixXd newN) {
  Eigen::MatrixXd concatV = Eigen::MatrixXd::Zero(Vertices.rows() + newV.rows(), Vertices.cols());
  concatV << Vertices, newV;
  Vertices = concatV;

  Eigen::MatrixXd concatN = Eigen::MatrixXd::Zero(Normals.rows() + newN.rows(), Normals.cols());
  concatN << Normals, newN;
  Normals = concatN;

  // Update point cloud
  //   - update octree
  //   - render points and normals
  updatePointCloud();
}

// Return the pointer to member variables
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
  double averageDistance = 0.0;
  
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
