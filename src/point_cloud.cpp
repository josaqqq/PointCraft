#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"

#include <igl/readOBJ.h>

#include <Eigen/Dense>

#include "point_cloud.hpp"
#include "constants.hpp"
#include "surface.hpp"

PointCloud::PointCloud(std::string filename) {
  std::cout << "loading: " << filename << std::endl;

  // Read the mesh
  Eigen::MatrixXd meshTC;   // double matrix of texture coordinates
  Eigen::MatrixXi meshFTC;  // list of face indices into vertex texture coordinates
  Eigen::MatrixXi meshFN;   // list of face indices into vertex normals
  igl::readOBJ(filename, meshV, meshTC, meshN, meshF, meshFTC, meshFN);
  std::cout << "Vertex num:\t\t"            << meshV.rows()   << std::endl;
  std::cout << "Texture coordinate num:\t"  << meshTC.rows()  << std::endl;
  std::cout << "Normal num:\t\t"            << meshN.rows()   << std::endl;
  std::cout << "Face num:\t\t"              << meshF.rows()   << std::endl;
  if (meshN.rows() == 0) std::cout << "ERROR: Please include normal information." << std::endl;

  // Move points to set the gravity point to (0.0, 0.0, 0.0).
  movePointsToOrigin();

  // Initialize octree
  octree = Octree(meshV, OctreeResolution);
  averageDistance = octree.calcAverageDistance();
  std::cout << "Average Distance:\t" << averageDistance << std::endl;

  // Update point cloud
  //   - update octree
  //   - render points and normals
  updatePointCloud();
} 

// Move points to set the gravity point to (0.0, 0.0, 0.0).
void PointCloud::movePointsToOrigin() {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  for (int i = 0; i < meshV.rows(); i++) {
    x += meshV(i, 0);
    y += meshV(i, 1);
    z += meshV(i, 2);
  }
  x /= static_cast<double>(meshV.rows());
  y /= static_cast<double>(meshV.rows());
  z /= static_cast<double>(meshV.rows());

  for (int i = 0; i < meshV.rows(); i++) {
    meshV(i, 0) -= x;
    meshV(i, 1) -= y;
    meshV(i, 2) -= z;
  }
}

// Update point cloud
//   - update octree
//   - render points and normals
void PointCloud::updatePointCloud() {
  // Update Octree
  octree = Octree(meshV, OctreeResolution);

  // Register Points
  polyscope::PointCloud* pointCloud = polyscope::registerPointCloud(PointName, meshV);
  pointCloud->setPointColor(PointColor);
  pointCloud->setPointRadius(PointRadius);

  // Register Normals
  polyscope::PointCloudVectorQuantity *vectorQuantity = pointCloud->addVectorQuantity(NormalName, meshN);
  vectorQuantity->setVectorColor(NormalColor);
  vectorQuantity->setVectorLengthScale(NormalLength);
  vectorQuantity->setVectorRadius(NormalRadius);
  vectorQuantity->setEnabled(NormalEnabled);
  vectorQuantity->setMaterial(NormalMaterial);

  // Reconstruct Surfaces
  poissonReconstruct(meshV, meshN);
  greedyProjection(meshV, meshN);
}

// Add points with information of the position and the normal.
void PointCloud::addPoints(Eigen::MatrixXd newV, Eigen::MatrixXd newN) {
  Eigen::MatrixXd concatV = Eigen::MatrixXd::Zero(meshV.rows() + newV.rows(), meshV.cols());
  concatV << meshV, newV;
  meshV = concatV;

  Eigen::MatrixXd concatN = Eigen::MatrixXd::Zero(meshN.rows() + newN.rows(), meshN.cols());
  concatN << meshN, newN;
  meshN = concatN;

  // Update point cloud
  //   - update octree
  //   - render points and normals
  updatePointCloud();
}