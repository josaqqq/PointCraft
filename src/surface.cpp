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
#include <random>

#include "surface.hpp"
#include "plane.hpp"
#include "ray.hpp"
#include "constants.hpp"

// Reconstruct new surface with Vertices and Normals and return them.
//  - averageDistance: used to decide the resolution of Poisson Surface Reconstruction
std::pair<std::vector<glm::dvec3>, std::vector<std::vector<size_t>>> Surface::reconstructPoissonSurface(double averageDistance) {
  // Init point cloud
  pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud(new pcl::PointCloud<pcl::PointNormal>);
  inputCloud->points.resize(Vertices->size());
  for (size_t i = 0; i < Vertices->size(); i++) {
    inputCloud->points[i].x = (*Vertices)[i].x;
    inputCloud->points[i].y = (*Vertices)[i].y;
    inputCloud->points[i].z = (*Vertices)[i].z;

    inputCloud->points[i].normal_x = (*Normals)[i].x;
    inputCloud->points[i].normal_y = (*Normals)[i].y;
    inputCloud->points[i].normal_z = (*Normals)[i].z;
  }

  // Calculate bounding box size
  const double INF = 1e5;
  double min_x = INF, max_x = -INF;
  double min_y = INF, max_y = -INF;
  double min_z = INF, max_z = -INF;
  for (size_t i = 0; i < Vertices->size(); i++) {
    min_x = std::min(min_x, (*Vertices)[i].x);
    max_x = std::max(max_x, (*Vertices)[i].x);
    min_y = std::min(min_y, (*Vertices)[i].y);
    max_y = std::max(max_y, (*Vertices)[i].y);
    min_z = std::min(min_z, (*Vertices)[i].z);
    max_z = std::max(max_z, (*Vertices)[i].z);
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
  std::vector<glm::dvec3> meshV(meshVertices->size());
  for (size_t i = 0; i < meshVertices->size(); i++) {
    meshV[i] = glm::dvec3(
      meshVertices->points[i].x,
      meshVertices->points[i].y,
      meshVertices->points[i].z
    );
  }

  // Extract face information
  std::vector<pcl::Vertices> meshFaces = mesh.polygons;
  std::vector<std::vector<size_t>> meshF(meshFaces.size(), std::vector<size_t>(3));
  for (size_t i = 0; i < meshFaces.size(); i++) {
    pcl::Vertices face = meshFaces[i];
    assert(face.vertices.size() == 3);
    for (size_t j = 0; j < face.vertices.size(); j++) {
      meshF[i][j] = face.vertices[j];
    }
  }

  // Register mesh
  polyscope::SurfaceMesh *poissonMesh = polyscope::registerSurfaceMesh(Name, meshV, meshF);
  poissonMesh->setSurfaceColor(PoissonColor);
  poissonMesh->setMaterial(PoissonMaterial);
  poissonMesh->setEnabled(PoissonEnabled);

  // Output results
  std::cout << "\nFinished Poisson Surface Reconstruction!"                   << std::endl;
  std::cout << "\tVoxel num\t->\t"            << voxelNum                     << std::endl;
  std::cout << "\tMax Depth\t->\t"            << maxDepth                     << std::endl;
  std::cout << "\tVertex num\t->\t"           << meshVertices->points.size()  << std::endl;
  std::cout << "\tFace num\t->\t"             << mesh.polygons.size()         << std::endl;
  std::cout << "\tInput Vertices Size\t->\t"  << Vertices->size()             << std::endl;
  std::cout << "\tInput Normals Size\t->\t"   << Normals->size()              << std::endl;
  std::cout << "\tOutput Vertices Size\t->\t" << meshV.size()                 << std::endl;
  std::cout << "\tOutput Faces Size\t->\t"    << meshF.size()                 << std::endl;
  std::cout                                                                   << std::endl;

  return { meshV, meshF };
}

// Compute approximate surface using Vertices and Normals.
// Then project points randomly onto the surface and return the projected points.
//  - xPos: io.DisplayFramebufferScale.x * mousePos.x
//  - xPos: io.DisplayFramebufferScale.y * mousePos.y
//  - averageDistance:  the range of the randomly added points.
//  - pointSize:        the size of randomly added points.
std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> Surface::projectMLSSurface(
  int xPos,
  int yPos,
  double averageDistance, 
  int pointSize
) {
  // Seed generation
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis(-averageDistance, averageDistance);
  
  // Calculate center point
  glm::dvec3 g(0.0, 0.0, 0.0);
  for (size_t i = 0; i < Vertices->size(); i++) g += (*Vertices)[i];
  g /= Vertices->size();
  Vertices->push_back(g);

  // Init point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
  inputCloud->points.resize(Vertices->size());
  for (size_t i = 0; i < Vertices->size(); i++) {
    inputCloud->points[i].x = (*Vertices)[i].x;
    inputCloud->points[i].y = (*Vertices)[i].y;
    inputCloud->points[i].z = (*Vertices)[i].z;
  } 

  // Create a KD-Tree 
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
  assert(inputCloud->points.size() == outputCloud->points.size());

  // Compute the center point where the ray 
  // intersects the approximate plane of MLS.
  std::vector<pcl::MLSResult> mlsResults = mls.getMLSResults();
  pcl::MLSResult mlsResult = mlsResults[Vertices->size() - 1];

  glm::dvec3 mlsCenter = glm::dvec3(mlsResult.mean.x(), mlsResult.mean.y(), mlsResult.mean.z());
  glm::dvec3 mlsPlaneNormal = glm::dvec3(mlsResult.plane_normal.x(), mlsResult.plane_normal.y(), mlsResult.plane_normal.z());
  glm::dvec3 mlsUAxis = glm::dvec3(mlsResult.u_axis.x(), mlsResult.u_axis.y(), mlsResult.u_axis.z());
  glm::dvec3 mlsVAxis = glm::dvec3(mlsResult.v_axis.x(), mlsResult.v_axis.y(), mlsResult.v_axis.z());
  
  Plane H(mlsCenter, mlsPlaneNormal);
  Ray ray(xPos, yPos);
  Ray::Hit hitInfo = ray.castPointToPlane(&H);
  if (!hitInfo.hit) return { std::vector<glm::dvec3>(), std::vector<glm::dvec3>() };
  
  double centerU = glm::dot(hitInfo.pos - mlsCenter, mlsUAxis);
  double centerV = glm::dot(hitInfo.pos - mlsCenter, mlsVAxis);

  // Project random points onto MLS surface
  std::vector<glm::dvec3> newV, newN;
  const glm::dvec3 cameraOrig = polyscope::view::getCameraWorldPosition();
  for (int i = 0; i < pointSize; i++) {
    double u = dis(gen);
    double v = dis(gen);

    pcl::MLSResult::MLSProjectionResults 
      projectedResult = mlsResult.projectPointSimpleToPolynomialSurface(centerU + u, centerV + v);
    
    glm::dvec3 p = glm::dvec3(
      projectedResult.point.x(),
      projectedResult.point.y(),
      projectedResult.point.z()
    );
    glm::dvec3 pn = glm::dvec3(
      projectedResult.normal.x(),
      projectedResult.normal.y(),
      projectedResult.normal.z()
    );

    // If the normal is not facing cameraOrig, then reverse it
    if (glm::dot(pn, p - cameraOrig) >= 0.0) pn *= -1.0;

    newV.push_back(p);
    newN.push_back(pn);
  }

  // Display points projected mls surface
  polyscope::PointCloud *mlsPoints = polyscope::registerPointCloud(Name, newV);
  mlsPoints->setPointRadius(PointRadius);
  mlsPoints->setEnabled(false);

  polyscope::PointCloudVectorQuantity *mlsVectorQuantity = mlsPoints->addVectorQuantity(NormalName, newN);
  mlsVectorQuantity->setVectorColor(NormalColor);
  mlsVectorQuantity->setVectorLengthScale(NormalLength);
  mlsVectorQuantity->setVectorRadius(NormalRadius);
  mlsVectorQuantity->setEnabled(NormalEnabled);

  return { newV, newN };
}

// Show hexagons for each vertex as a pseudo surface.
//  - averageDistance:  the radius of the shown hexagon.
void Surface::showPseudoSurface(double averageDistance) {
  size_t N = Vertices->size();
  std::vector<glm::dvec3> meshV(N*7);
  std::vector<std::vector<size_t>> meshF(N*6);

  // Calculate rotation matrix
  double angleInDegrees = 60.0d;
  double angleInRadians = glm::radians(angleInDegrees);
  glm::dmat4 rot = glm::rotate(angleInRadians, glm::dvec3(0.0, 0.0, 1.0));

  // Register each hexagon
  for (size_t i = 0; i < N; i++) {
    glm::dvec3 o = (*Vertices)[i];
    glm::dvec3 n = (*Normals)[i];

    Plane plane(o, n);
    meshV[i] = o;

    // Register vertex point
    glm::dvec4 hex = glm::dvec4(averageDistance, 0.0, 0.0, 0.0);
    for (int j = 0; j < 6; j++) {
      meshV[(j + 1)*N + i] = plane.unmapCoordinates(glm::dvec3(hex.x, hex.y, hex.z));
      hex = rot*hex;
    }

    // Register face index
    for (size_t j = 0; j < 6; j++) {
      meshF[i*6 + j] = {
        i,
        (j + 1)*N + i,
        ((j + 1)%6 + 1)*N + i
      };
    }
  }

  // Register mesh
  polyscope::SurfaceMesh *pseudoSurface = polyscope::registerSurfaceMesh(Name, meshV, meshF);
  pseudoSurface->setSurfaceColor(PseudoSurfaceColor);
  pseudoSurface->setMaterial(PseudoSurfaceMaterial);
}
