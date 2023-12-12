#include "polyscope/polyscope.h"

#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"
#include "polyscope/view.h"

#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
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
std::pair<std::vector<glm::dvec3>, std::vector<std::vector<size_t>>> Surface::reconstructPoissonSurface(
  double averageDistance,
  bool enabled
) {
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
  poissonMesh->setBackFaceColor(PoissonBackFaceColor);
  poissonMesh->setMaterial(PoissonMaterial);
  poissonMesh->setBackFacePolicy(PoissonBackFacePolicy);
  poissonMesh->setEnabled(enabled);

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

// Compute an approximate surface using Vertices and Normals.
// Then project points randomly onto the surface and return the projected points.
//  - xPos: io.DisplayFramebufferScale.x * mousePos.x
//  - xPos: io.DisplayFramebufferScale.y * mousePos.y
//  - searchRadius: the range of the nearest neighbor search
//  - averageDistance:  the radius of the range where points are randomly added.
//  - pointSize:        the size of randomly added points.
std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> Surface::projectMLSSurface(
  int xPos,
  int yPos,
  double searchRadius,
  double averageDistance, 
  int pointSize
) {
  // Initialize MLS surface
  pcl::MLSResult mlsResult = InitializeMLSSurface(searchRadius);

  glm::dvec3 mlsCenter = glm::dvec3(mlsResult.mean.x(), mlsResult.mean.y(), mlsResult.mean.z());
  glm::dvec3 mlsPlaneNormal = glm::dvec3(mlsResult.plane_normal.x(), mlsResult.plane_normal.y(), mlsResult.plane_normal.z());
  glm::dvec3 mlsUAxis = glm::dvec3(mlsResult.u_axis.x(), mlsResult.u_axis.y(), mlsResult.u_axis.z());
  glm::dvec3 mlsVAxis = glm::dvec3(mlsResult.v_axis.x(), mlsResult.v_axis.y(), mlsResult.v_axis.z());
  
  // Cast a ray to the approximate plane
  Plane H(mlsCenter, mlsPlaneNormal);
  Ray ray(xPos, yPos);
  Ray::Hit hitInfo = ray.castPointToPlane(&H);
  if (!hitInfo.hit) return { std::vector<glm::dvec3>(), std::vector<glm::dvec3>() };
  
  double centerU = glm::dot(hitInfo.pos - mlsCenter, mlsUAxis);
  double centerV = glm::dot(hitInfo.pos - mlsCenter, mlsVAxis);

  // Project random points onto MLS surface
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis(-averageDistance, averageDistance);
  
  std::vector<glm::dvec3> newV, newN;
  const glm::dvec3 cameraOrig = polyscope::view::getCameraWorldPosition();
  for (int i = 0; i < pointSize; i++) {
    double u = centerU + dis(gen);
    double v = centerV + dis(gen);

    pcl::MLSResult::MLSProjectionResults 
      projectedResult = mlsResult.projectPointSimpleToPolynomialSurface(u, v);
    
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

// Compute an approximate surface using Vertices and Normals.
// Then project points inside of the range of basis points.
//  - searchRadius: the range of the nearest neighbor search
//  - averagedistance: the distance between points in the point cloud
std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> Surface::reconstructMLSSurface(
  double searchRadius,
  double averageDistance
) {
  // Initialize MLS Surface
  pcl::MLSResult mlsResult = InitializeMLSSurface(searchRadius);

  glm::dvec3 mlsCenter = glm::dvec3(mlsResult.mean.x(), mlsResult.mean.y(), mlsResult.mean.z());
  glm::dvec3 mlsPlaneNormal = glm::dvec3(mlsResult.plane_normal.x(), mlsResult.plane_normal.y(), mlsResult.plane_normal.z());
  glm::dvec3 mlsUAxis = glm::dvec3(mlsResult.u_axis.x(), mlsResult.u_axis.y(), mlsResult.u_axis.z());
  glm::dvec3 mlsVAxis = glm::dvec3(mlsResult.v_axis.x(), mlsResult.v_axis.y(), mlsResult.v_axis.z());

  // Cast all vertices to the MLS surface
  Plane H(mlsCenter, mlsPlaneNormal);
  const double INF = 1e5;
  double min_x = INF, max_x = -INF;
  double min_y = INF, max_y = -INF;
  for (size_t i = 0; i < Vertices->size(); i++) {
    glm::dvec3 mappedPoint = H.mapCoordinates((*Vertices)[i]);

    min_x = std::min(min_x, mappedPoint.x);
    max_x = std::max(max_x, mappedPoint.x);
    min_y = std::min(min_y, mappedPoint.y);
    max_y = std::max(max_y, mappedPoint.y);
  }

  // Project discretized point to the MLS surface
  // Discretize in squares with averageDistance/2.0 on each side
  std::vector<glm::dvec3> newV, newN;
  const glm::dvec3 cameraOrig = polyscope::view::getCameraWorldPosition();
  for (double x = min_x; x < max_x; x += averageDistance/2.0) {
    for (double y = min_y; y < max_y; y += averageDistance/2.0) {
      pcl::MLSResult::MLSProjectionResults 
        projectedResult = mlsResult.projectPointSimpleToPolynomialSurface(x, y);
      
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

      // If the normal is not facing cameraOrig, then skip it
      if (glm::dot(pn, p - cameraOrig) >= 0.0) continue;

      newV.push_back(p);
      newN.push_back(pn);
    }
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

void Surface::showGreedyProjection(
  double averageDistance,
  bool enabled
) {
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

  // Initialize kd-tree
  pcl::search::KdTree<pcl::PointNormal>::Ptr kdTree(new pcl::search::KdTree<pcl::PointNormal>);
  kdTree->setInputCloud(inputCloud);

  // Initialize greedy projection object
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gpt;
  gpt.setInputCloud(inputCloud);
  gpt.setSearchMethod(kdTree);

  // Set parameters
  gpt.setSearchRadius(averageDistance*GreedyProjSearchRadiusMult);  // The sphere radius that is to be used for determining the k-nearest neighbors used for triangulating.
  gpt.setMu(GreedyProjMu);                                          // The multiplier of the nearest neighbor distance to obtain the final search radius for each point (this will make the algorithm adapt to different point densities in the cloud).
  gpt.setMaximumNearestNeighbors(GreedyProjMaxNN);                  // The maximum number of nearest neighbors to be searched for.
  gpt.setMaximumSurfaceAngle(GreedyProjMaxSurfaceAngle);            // Don't consider points for triangulation if their normal deviates more than this value from the query point's normal.
  gpt.setMinimumAngle(GreedyProjMinAngle);                          // The minimum angle each triangle should have.
  gpt.setMaximumAngle(GreedyProjMaxAngle);                          // Maximum angle each triangle can have.
  gpt.setNormalConsistency(GreedyProjNormalConsistency);            // The flag if the input normals are oriented consistently.
  gpt.setConsistentVertexOrdering(GreedyProjVertexConsistency);     // The flag to order the resulting triangle vertices consistently (positive direction around normal).

  // Reconstruct surface
  pcl::PolygonMesh mesh;
  gpt.reconstruct(mesh);

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

  // Coloring hole boundary
  // TODO: Implement here

  // Register mesh
  polyscope::SurfaceMesh *greedyMesh = polyscope::registerSurfaceMesh(Name, meshV, meshF);
  greedyMesh->setSurfaceColor(GreedyProjColor);
  greedyMesh->setBackFaceColor(GreedyProjBackFaceColor);
  greedyMesh->setBackFacePolicy(GreedyProjBackFacePolicy);
  greedyMesh->setMaterial(GreedyProjMaterial);
  greedyMesh->setEnabled(enabled);

  // Output results
  std::cout << "\nFinished Greedy Projection Triangulation!"  << std::endl;
  std::cout << "\tVertex num\t->\t" << meshV.size()           << std::endl;
  std::cout << "\tFace num\t->\t"   << meshF.size()           << std::endl;
  std::cout                                                   << std::endl;
}

// Show hexagons for each vertex as a pseudo surface.
//  - averageDistance:  the radius of the shown hexagon.
//  - enabled:  If true, enable the registered pseudo surface
void Surface::showPseudoSurface(double averageDistance, bool enabled) {
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
  pseudoSurface->setBackFaceColor(PseudoSurfaceBackFaceColor);
  pseudoSurface->setMaterial(PseudoSurfaceMaterial);
  pseudoSurface->setBackFacePolicy(PseudoSurfaceBackFacePolicy);
  pseudoSurface->setEnabled(enabled);
}

pcl::MLSResult Surface::InitializeMLSSurface(double searchRadius) {
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
  mls.setSearchRadius(searchRadius);

  // Execute MLS
  pcl::PointCloud<pcl::PointNormal>::Ptr outputCloud(new pcl::PointCloud<pcl::PointNormal>);
  mls.process(*outputCloud);
  assert(inputCloud->points.size() == outputCloud->points.size());

  // Compute the center point where the ray 
  // intersects the approximate plane of MLS.
  std::vector<pcl::MLSResult> mlsResults = mls.getMLSResults();
  pcl::MLSResult mlsResult = mlsResults[Vertices->size() - 1];

  return mlsResult;
}