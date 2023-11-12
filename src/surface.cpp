#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"

#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>

#include <string>

// Parameters
// Poisson Surface Reconstruction
const int PoissonDepth = 5;
const glm::vec3 PoissonColor = { 0.155, 0.186, 0.790 };
const std::string PoissonMaterial = "normal";

// Moving Least Squares
const bool MLSPolynogmialFitFlag = true;
const double MLSSearchRadius = 0.3;

// Delaunay Triangulation
const double GreedyProjSearchRadius = 0.5;
const double GreedyProjMu = 2.5;
const int GreedyProjMaxNN = 100;
const double GreedyProjMaxSurfaceAngle = M_PI/4.0; // 45 degrees
const double GreedyProjMinAngle = M_PI/18.0;       // 10 degrees
const double GreedyProjMaxAngle = 2.0*M_PI/3.0;    // 120 degrees
const bool GreedyProjNormalConsistency = false;

const glm::vec3 GreedyProjColor = { 0.155, 0.186, 0.790 };
const std::string GreedyProjMaterial = "normal";

// Reconstruct surface with vertex and normal infromation
void poissonReconstruct(Eigen::MatrixXd vertices, Eigen::MatrixXd normals) {
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

  // Initialize poisson surface reconstruction
  pcl::Poisson<pcl::PointNormal> poisson;
  poisson.setDepth(PoissonDepth);
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
  std::cout << "Vertex num:\t" << meshVertices->points.size() << std::endl;
  std::cout << "Face num:\t" << mesh.polygons.size() << std::endl;

  // Register mesh
  polyscope::SurfaceMesh *surfaceMesh = polyscope::registerSurfaceMesh("Poisson Surface Reconstruction", meshV, meshF);
  surfaceMesh->setSurfaceColor(PoissonColor);
  surfaceMesh->setMaterial(PoissonMaterial);
}

// Smoothing vertices with MLS
void mlsSmoothing(Eigen::MatrixXd vertices) {
  // Init point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
  inputCloud->points.resize(vertices.rows());
  for (int i = 0; i < vertices.rows(); i++) {
    inputCloud->points[i].x = vertices(i, 0);
    inputCloud->points[i].y = vertices(i, 1);
    inputCloud->points[i].z = vertices(i, 2);
  } 

  // Initialize Moving Least Squares
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
  // TODO: We can use Kdtree here.
  mls.setInputCloud(inputCloud);
  mls.setPolynomialFit(MLSPolynogmialFitFlag);
  mls.setSearchRadius(MLSSearchRadius);

  // Execute MLS
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>());
  mls.process(*outputCloud);

  Eigen::MatrixXd meshV(outputCloud->size(), 3);
  for (int i = 0; i < outputCloud->size(); i++) {
    meshV.row(i) << 
      outputCloud->points[i].x,
      outputCloud->points[i].y,
      outputCloud->points[i].z;
  }

  polyscope::PointCloud *mlsPoints = polyscope::registerPointCloud("MLS Points", meshV);
}

void greedyProjection(Eigen::MatrixXd vertices, Eigen::MatrixXd normals) {
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
  polyscope::SurfaceMesh *surfaceMesh = polyscope::registerSurfaceMesh("Greedy Projection Triangulation", meshV, meshF);
  surfaceMesh->setSurfaceColor(GreedyProjColor);
  surfaceMesh->setMaterial(GreedyProjMaterial);
}