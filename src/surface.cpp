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

#include <glm/gtx/transform.hpp>

#include <string>

#include "surface.hpp"
#include "plane.hpp"
#include "constants.hpp"

// Reconstruct surface with vertex and normal infromation
std::pair<Eigen::MatrixXd, Eigen::MatrixXi> poissonReconstruct(
  std::string name,
  double averageDistance,
  Eigen::MatrixXd &vertices,
  Eigen::MatrixXd &normals
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
  // Grid the bounding box with voxels ((averageDistance/2.0)^3)
  double voxelEdge = averageDistance / 2.0;
  int voxelNum = ((max_x - min_x)*(max_y - min_y)*(max_z - min_z))/(voxelEdge*voxelEdge*voxelEdge);

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

  // Register mesh
  polyscope::SurfaceMesh *poissonMesh = polyscope::registerSurfaceMesh(name, meshV, meshF);
  poissonMesh->setSurfaceColor(PoissonColor);
  poissonMesh->setMaterial(PoissonMaterial);
  poissonMesh->setEnabled(PoissonEnabled);

  // Output results
  std::cout << "\nFinished Poisson Surface Reconstruction!"                   << std::endl;
  std::cout << "\tVoxel num\t->\t"            << voxelNum                     << std::endl;
  std::cout << "\tMax Depth\t->\t"            << maxDepth                     << std::endl;
  std::cout << "\tVertex num\t->\t"           << meshVertices->points.size()  << std::endl;
  std::cout << "\tFace num\t->\t"             << mesh.polygons.size()         << std::endl;
  std::cout << "\tInput Vertices Size\t->\t"  << vertices.rows()              << std::endl;
  std::cout << "\tInput Normals Size\t->\t"   << normals.rows()               << std::endl;
  std::cout << "\tOutput Vertices Size\t->\t" << meshV.rows()                 << std::endl;
  std::cout << "\tOutput Faces Size\t->\t"    << meshF.rows()                 << std::endl;
  std::cout                                                                   << std::endl;

  return { meshV, meshF };
}

// Smoothing vertices with MLS
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> mlsSmoothing(
  std::string name,
  Eigen::MatrixXd &vertices
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
  mls.setSearchMethod(tree);
  mls.setSearchRadius(MLSSearchRadius);

  // Execute MLS
  pcl::PointCloud<pcl::PointNormal>::Ptr outputCloud(new pcl::PointCloud<pcl::PointNormal>);
  mls.process(*outputCloud);

  Eigen::MatrixXd Vertices(outputCloud->size(), 3);
  Eigen::MatrixXd Normals(outputCloud->size(), 3);
  glm::dvec3 averageNormal = glm::dvec3(0.0, 0.0, 0.0);
  for (int i = 0; i < outputCloud->size(); i++) {
    Vertices.row(i) << 
      outputCloud->points[i].x,
      outputCloud->points[i].y,
      outputCloud->points[i].z;
    
    Normals.row(i) <<
      outputCloud->points[i].normal_x,
      outputCloud->points[i].normal_y,
      outputCloud->points[i].normal_z;
    
    averageNormal += glm::dvec3(Normals(i, 0), Normals(i, 1), Normals(i, 2));
  }
  averageNormal /= (double)outputCloud->size();
  
  // If the averageNormal is the same direction with cameraDir, flip the normals
  const glm::dvec3 cameraDir  = polyscope::view::screenCoordsToWorldRay(glm::vec2(polyscope::view::windowWidth/2, polyscope::view::windowHeight/2));
  if (glm::dot(averageNormal, cameraDir) > 0) Normals *= -1;

  polyscope::PointCloud *mlsPoints = polyscope::registerPointCloud(name, Vertices);
  mlsPoints->setPointColor(PointColor);
  mlsPoints->setPointRadius(PointRadius);
  mlsPoints->setEnabled(PointEnabled);

  polyscope::PointCloudVectorQuantity *mlsVectorQuantity = mlsPoints->addVectorQuantity(NormalName, Normals);
  mlsVectorQuantity->setVectorColor(NormalColor);
  mlsVectorQuantity->setVectorLengthScale(NormalLength);
  mlsVectorQuantity->setVectorRadius(NormalRadius);
  mlsVectorQuantity->setEnabled(NormalEnabled);

  return { Vertices, Normals };
}

void greedyProjection(
  std::string name,
  Eigen::MatrixXd &vertices,
  Eigen::MatrixXd &normals
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
  std::cout << "\tVertex num\t->\t" << meshVertices->points.size() << std::endl;
  std::cout << "\tFace num\t->\t" << mesh.polygons.size() << std::endl;

  // Register mesh
  polyscope::SurfaceMesh *greedyMesh = polyscope::registerSurfaceMesh(name, meshV, meshF);
  greedyMesh->setSurfaceColor(GreedyProjColor);
  greedyMesh->setMaterial(GreedyProjMaterial);
  greedyMesh->setEnabled(GreedyProjEnabled);
}

// Show hexagons for each vertex as a pseudo surface.
void pseudoSurface(
  std::string name,
  double averageDistance,
  Eigen::MatrixXd &vertices,
  Eigen::MatrixXd &normals
) {
  int N = vertices.rows();
  Eigen::MatrixXd meshV(N*7, 3);
  Eigen::MatrixXd meshF(N*6, 3);

  // Calculate rotation matrix
  double angleInDegrees = 60.0d;
  double angleInRadians = glm::radians(angleInDegrees);
  glm::dmat4 rot = glm::rotate(angleInRadians, glm::dvec3(0.0, 0.0, 1.0));

  // Register each hexagon
  for (int i = 0; i < N; i++) {
    glm::dvec3 o = glm::dvec3(vertices(i, 0), vertices(i, 1), vertices(i, 2));
    glm::dvec3 n = glm::dvec3(normals(i, 0), normals(i, 1), normals(i, 2));

    Plane plane(o, n);
    meshV.row(i) << o.x, o.y, o.z;
    
    // Register vertex point
    glm::dvec4 hex = glm::dvec4(averageDistance, 0.0, 0.0, 0.0);
    for (int j = 0; j < 6; j++) {
      glm::dvec3 hexInWorld = plane.unmapCoordinates(glm::dvec3(hex.x, hex.y, hex.z));
      meshV.row((j + 1)*N + i) << hexInWorld.x, hexInWorld.y, hexInWorld.z;
      hex = rot*hex;
    }

    // Register face index
    for (int j = 0; j < 6; j++) {
      meshF.row(i*6 + j) << i, (j + 1)*N + i, ((j + 1)%6 + 1)*N + i;
    }
  }

  // Register mesh
  polyscope::SurfaceMesh *pseudoSurface = polyscope::registerSurfaceMesh(name, meshV, meshF);
  pseudoSurface->setSurfaceColor(PseudoSurfaceColor);
  pseudoSurface->setMaterial(PseudoSurfaceMaterial);
}