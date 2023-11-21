#include "polyscope/polyscope.h"

#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"
#include "polyscope/view.h"

#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>

#include <string>

#include "surface.hpp"
#include "constants.hpp"

// Reconstruct surface with vertex and normal infromation
void poissonReconstruct(
  std::string name,
  double averageDistance,
  Eigen::MatrixXd vertices,
  Eigen::MatrixXd normals
) {
  // Init point cloud
  pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud(new pcl::PointCloud<pcl::PointNormal>);
  inputCloud->points.resize(vertices.rows());
  for (int i = 0; i < vertices.rows(); i++) {
    inputCloud->points[i].x = vertices(i, 0);
    inputCloud->points[i].y = vertices(i, 1);
    inputCloud->points[i].z = vertices(i, 2);

    inputCloud->points[i].normal_x = normals(i, 0);
    inputCloud->points[i].normal_y = normals(i, 1);
    inputCloud->points[i].normal_z = normals(i, 2);
  }

  // Calculate bounding box size
  const double INF = 1e5;
  double min_x = INF, max_x = -INF;
  double min_y = INF, max_y = -INF;
  double min_z = INF, max_z = -INF;
  for (int i = 0; i < vertices.rows(); i++) {
    min_x = std::min(min_x, vertices(i, 0));
    max_x = std::max(max_x, vertices(i, 0));
    min_y = std::min(min_y, vertices(i, 1));
    max_y = std::max(max_y, vertices(i, 1));
    min_z = std::min(min_z, vertices(i, 2));
    max_z = std::max(max_z, vertices(i, 2));
  }
  // Grid the bounding box with voxels (averageDistance^3)
  int voxelNum = ((max_x - min_x)*(max_y - min_y)*(max_z - min_z))/(averageDistance*averageDistance*averageDistance);

  int maxDepth = PoissonMaxDepth;
  for (int i = 0; i <= PoissonMaxDepth; i++) {
    if (voxelNum <= pow(2, i*3)) {
      maxDepth = i;
      break;
    }
  }

  // Initialize poisson surface reconstruction
  pcl::Poisson<pcl::PointNormal> poisson;
  poisson.setDepth(maxDepth);
  poisson.setInputCloud(inputCloud);

  // Reconstruct surface
  pcl::PolygonMesh mesh;
  poisson.performReconstruction(mesh);

  // Extract vertex information
  pcl::PointCloud<pcl::PointXYZ>::Ptr meshVertices(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *meshVertices);
  Eigen::MatrixXd meshV(meshVertices->size(), 3);

  for (int i = 0; i < meshVertices->size(); i++) {
    meshV.row(i) << 
      meshVertices->points[i].x,
      meshVertices->points[i].y,
      meshVertices->points[i].z;
  }

  // Extract face information
  std::vector<pcl::Vertices> meshFaces = mesh.polygons;
  Eigen::MatrixXi meshF(meshFaces.size(), 3);

  for (int i = 0; i < meshFaces.size(); i++) {
    pcl::Vertices face = meshFaces[i];
    assert(face.vertices.size() == 3);
    for (int j = 0; j < face.vertices.size(); j++) {
      meshF(i, j) = face.vertices[j];
    }
  }

  // Output results
  std::cout << "\nFinished Poisson Surface Reconstruction!" << std::endl;
  std::cout << "Max Depth:\t" << maxDepth << std::endl;
  std::cout << "Vertex num:\t" << meshVertices->points.size() << std::endl;
  std::cout << "Face num:\t" << mesh.polygons.size() << std::endl;

  // Register mesh
  polyscope::SurfaceMesh *surfaceMesh = polyscope::registerSurfaceMesh(name, meshV, meshF);
  surfaceMesh->setSurfaceColor(PoissonColor);
  surfaceMesh->setMaterial(PoissonMaterial);
}

// Smoothing vertices with MLS
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> mlsSmoothing(
  std::string name,
  Eigen::MatrixXd vertices
) {
  // Init point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
  inputCloud->points.resize(vertices.rows());
  for (int i = 0; i < vertices.rows(); i++) {
    inputCloud->points[i].x = vertices(i, 0);
    inputCloud->points[i].y = vertices(i, 1);
    inputCloud->points[i].z = vertices(i, 2);
  } 

  // Create a KD-Tree 
  // TODO: Not KD-Tree but Octree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  // Initialize Moving Least Squares
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals(true);
  mls.setInputCloud(inputCloud);
  mls.setPolynomialOrder(MLSPolynomialOrder);
  mls.setPolynomialFit(MLSPolynomialFitFlag);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(MLSSearchRadius);

  // Execute MLS
  pcl::PointCloud<pcl::PointNormal>::Ptr outputCloud(new pcl::PointCloud<pcl::PointNormal>);
  mls.process(*outputCloud);

  Eigen::MatrixXd meshV(outputCloud->size(), 3);
  Eigen::MatrixXd meshN(outputCloud->size(), 3);
  glm::dvec3 averageNormal = glm::dvec3(0.0, 0.0, 0.0);
  for (int i = 0; i < outputCloud->size(); i++) {
    meshV.row(i) << 
      outputCloud->points[i].x,
      outputCloud->points[i].y,
      outputCloud->points[i].z;
    
    meshN.row(i) <<
      outputCloud->points[i].normal_x,
      outputCloud->points[i].normal_y,
      outputCloud->points[i].normal_z;
    
    averageNormal += glm::dvec3(meshN(i, 0), meshN(i, 1), meshN(i, 2));
  }
  averageNormal /= (double)outputCloud->size();
  
  // If the averageNormal is the same direction with cameraDir, flip the normals
  const glm::dvec3 cameraDir  = polyscope::view::screenCoordsToWorldRay(glm::vec2(polyscope::view::windowWidth/2, polyscope::view::windowHeight/2));
  if (glm::dot(averageNormal, cameraDir) > 0) meshN *= -1;

  polyscope::PointCloud *mlsPoints = polyscope::registerPointCloud(name, meshV);
  mlsPoints->setPointColor(PointColor);
  mlsPoints->setPointRadius(PointRadius);

  polyscope::PointCloudVectorQuantity *mlsVectorQuantity = mlsPoints->addVectorQuantity(NormalName, meshN);
  mlsVectorQuantity->setVectorColor(NormalColor);
  mlsVectorQuantity->setVectorLengthScale(NormalLength);
  mlsVectorQuantity->setVectorRadius(NormalRadius);
  mlsVectorQuantity->setEnabled(NormalEnabled);

  return { meshV, meshN };
}

void greedyProjection(
  std::string name,
  Eigen::MatrixXd vertices,
  Eigen::MatrixXd normals
) {
  // Init point cloud
  pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud(new pcl::PointCloud<pcl::PointNormal>);
  inputCloud->points.resize(vertices.rows());
  for (int i = 0; i < vertices.rows(); i++) {
    inputCloud->points[i].x = vertices(i, 0);
    inputCloud->points[i].y = vertices(i, 1);
    inputCloud->points[i].z = vertices(i, 2);
    
    inputCloud->points[i].normal_x = normals(i, 0);
    inputCloud->points[i].normal_y = normals(i, 1);
    inputCloud->points[i].normal_z = normals(i, 2);
  }

  // Initialize kd-tree
  pcl::search::KdTree<pcl::PointNormal>::Ptr kdTree(new pcl::search::KdTree<pcl::PointNormal>);
  kdTree->setInputCloud(inputCloud);

  // Initialize greedy projection object
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gpt;
  gpt.setInputCloud(inputCloud);
  gpt.setSearchMethod(kdTree);

  // Set parameters
  gpt.setSearchRadius(GreedyProjSearchRadius);
  gpt.setMu(GreedyProjMu);
  gpt.setMaximumNearestNeighbors(GreedyProjMaxNN);
  gpt.setMaximumSurfaceAngle(GreedyProjMaxSurfaceAngle);
  gpt.setMinimumAngle(GreedyProjMinAngle);
  gpt.setMaximumAngle(GreedyProjMaxAngle);
  gpt.setNormalConsistency(GreedyProjNormalConsistency);

  // Reconstruct surface
  pcl::PolygonMesh mesh;
  gpt.reconstruct(mesh);

  // Extract vertex information
  pcl::PointCloud<pcl::PointXYZ>::Ptr meshVertices(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *meshVertices);
  Eigen::MatrixXd meshV(meshVertices->size(), 3);

  for (int i = 0; i < meshVertices->size(); i++) {
    meshV.row(i) << 
      meshVertices->points[i].x,
      meshVertices->points[i].y,
      meshVertices->points[i].z;
  }

  // Extract face information
  std::vector<pcl::Vertices> meshFaces = mesh.polygons;
  Eigen::MatrixXi meshF(meshFaces.size(), 3);

  for (int i = 0; i < meshFaces.size(); i++) {
    pcl::Vertices face = meshFaces[i];
    assert(face.vertices.size() == 3);
    for (int j = 0; j < face.vertices.size(); j++) {
      meshF(i, j) = face.vertices[j];
    }
  }

  // Output results
  std::cout << "\nFinished Greedy Projection Triangulation!" << std::endl;
  std::cout << "Vertex num:\t" << meshVertices->points.size() << std::endl;
  std::cout << "Face num:\t" << mesh.polygons.size() << std::endl;

  // Register mesh
  polyscope::SurfaceMesh *surfaceMesh = polyscope::registerSurfaceMesh(name, meshV, meshF);
  surfaceMesh->setSurfaceColor(GreedyProjColor);
  surfaceMesh->setMaterial(GreedyProjMaterial);
}