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
#include <map>
#include <set>

#include "surface.hpp"
#include "plane.hpp"
#include "ray.hpp"
#include "constants.hpp"

// Reconstruct new surface with Vertices and Normals.
//  - name: The name of the registered surface
//  - averageDistance: used to decide the resolution of Poisson Surface Reconstruction
//  - enabled:  If true, enable the registered poisson surface
std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> Surface::reconstructPoissonSurface(
  std::string name,
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

  // Initialize poisson surface reconstruction
  pcl::Poisson<pcl::PointNormal> poisson;
  poisson.setDepth(PoissonMaxDepth);
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

  // Calculate the points' average normals
  std::map<int, glm::dvec3> indexToNormalSum;
  std::map<int, int>        indexToAdjacentCount;
  // Normals of meshV
  for (size_t i = 0; i < meshF.size(); i++) {
    // Calculate the normal of the triangle
    glm::dvec3 u = meshV[meshF[i][0]];
    glm::dvec3 v = meshV[meshF[i][1]];
    glm::dvec3 w = meshV[meshF[i][2]];

    glm::dvec3 n = glm::normalize(glm::cross(v - u, w - u));
    for (int j = 0; j < 3; j++) {
      int vertexIdx = meshF[i][j];
      indexToNormalSum[vertexIdx] += n;
      indexToAdjacentCount[vertexIdx]++;
    }
  }

  std::vector<glm::dvec3> meshN(meshV.size());
  for (size_t i = 0; i < meshV.size(); i++) {
    meshN[i] = indexToNormalSum[i]/(double)indexToAdjacentCount[i];
  }

  // Register mesh
  polyscope::SurfaceMesh *poissonMesh = polyscope::registerSurfaceMesh(name, meshV, meshF);
  poissonMesh->setSurfaceColor(PoissonColor);
  poissonMesh->setBackFaceColor(PoissonBackFaceColor);
  poissonMesh->setMaterial(PoissonMaterial);
  poissonMesh->setBackFacePolicy(PoissonBackFacePolicy);
  poissonMesh->setEnabled(enabled);

  // Output results
  std::cout << "\nFinished Poisson Surface Reconstruction!"                   << std::endl;
  std::cout << "\tVertex num\t->\t"           << meshVertices->points.size()  << std::endl;
  std::cout << "\tFace num\t->\t"             << mesh.polygons.size()         << std::endl;
  std::cout << "\tInput Vertices Size\t->\t"  << Vertices->size()             << std::endl;
  std::cout << "\tInput Normals Size\t->\t"   << Normals->size()              << std::endl;
  std::cout << "\tOutput Vertices Size\t->\t" << meshV.size()                 << std::endl;
  std::cout << "\tOutput Faces Size\t->\t"    << meshF.size()                 << std::endl;
  std::cout                                                                   << std::endl;

  return { meshV, meshN };
}

// Compute an approximate surface using Vertices and Normals.
// Then project points randomly onto the surface and return the projected points.
//  - name: The name of the registered surface 
//  - mousePos: screen coordinates of the mouse position
//  - searchRadius: the range of the nearest neighbor search
//  - averageDistance:  the radius of the range where points are randomly added.
//  - pointSize:        the size of randomly added points.
std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> Surface::projectMLSSurface(
  std::string name,
  glm::dvec2 mousePos,
  double searchRadius,
  double averageDistance, 
  int pointSize
) {
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

  glm::dvec3 mlsCenter = glm::dvec3(mlsResult.mean.x(), mlsResult.mean.y(), mlsResult.mean.z());
  glm::dvec3 mlsPlaneNormal = glm::dvec3(mlsResult.plane_normal.x(), mlsResult.plane_normal.y(), mlsResult.plane_normal.z());
  glm::dvec3 mlsUAxis = glm::dvec3(mlsResult.u_axis.x(), mlsResult.u_axis.y(), mlsResult.u_axis.z());
  glm::dvec3 mlsVAxis = glm::dvec3(mlsResult.v_axis.x(), mlsResult.v_axis.y(), mlsResult.v_axis.z());
  
  // Cast a ray to the approximate plane
  Plane H(mlsCenter, mlsPlaneNormal);
  Ray ray(mousePos.x, mousePos.y);
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
  polyscope::PointCloud *mlsPoints = polyscope::registerPointCloud(name, newV);
  mlsPoints->setPointRadius(PointRadius);
  mlsPoints->setEnabled(false);

  polyscope::PointCloudVectorQuantity *mlsVectorQuantity = mlsPoints->addVectorQuantity(NormalName, newN);
  mlsVectorQuantity->setVectorColor(NormalColor);
  mlsVectorQuantity->setVectorLengthScale(NormalLength);
  mlsVectorQuantity->setVectorRadius(NormalRadius);
  mlsVectorQuantity->setEnabled(NormalEnabled);

  return { newV, newN };
}

// Render greedy surface and pseudo surface.
// Pseudo surface around holes in greedy surface is colored red.
// Return the number of points on hole boundary.
//  - greedyName: The name for greedy surface
//  - pseudoname: The name for pseudo surface
//  - averageDistance:  
//      For greedy surface, used to determine the search radius
//      For pseudo surface, the radius of the shown hexagons
//  - greedyEnabled: If true, enable the registered greedy surface
//  - pseudoEnabled: If true, enable the registered pseudo surface
int Surface::renderPointCloudSurface(
  std::string greedyName,
  std::string pseudoName,
  double averageDistance, 
  bool greedyEnabled,
  bool pseudoEnabled
) {
  // Greedy Surface
  std::set<int> boundaryVerticesIdx = renderGreedySurface(greedyName, averageDistance, greedyEnabled);
  // Pseudo Surface
  renderPseudoSurface(pseudoName, averageDistance, pseudoEnabled, boundaryVerticesIdx);

  return boundaryVerticesIdx.size();
}

// Compute Greedy Projection and then render the reconstructed mesh.
// Return the vertex indices on the hole boundaries.
//  - name: The name of the registered surface
//  - averageDistance:  used to determin the search radius
//  - enabled:  If true, enable the registered greedy surface
std::set<int> Surface::renderGreedySurface(
  std::string name,
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

  // Detect holes on the greedy projection surface and determine face colors
  std::set<int> holeBoundaryFacesIdx = detectHolesOnMesh(meshV, meshF, averageDistance*GreedyProjHoleDetectMult);

  std::set<int> boundaryVerticesIdx;
  std::vector<glm::dvec3> meshFC(meshF.size());
  for (size_t i = 0; i < meshF.size(); i++) {
    if (holeBoundaryFacesIdx.count(i)) {
      // On the boundary
      for (int idx: meshF[i]) boundaryVerticesIdx.insert(idx);
      meshFC[i] = GreedyProjBoundaryColor;
    } else {
      // Not on the boundary
      meshFC[i] = GreedyProjColor;
    }
  }

  // Register mesh
  polyscope::SurfaceMesh *greedyMesh = polyscope::registerSurfaceMesh(name, meshV, meshF);
  greedyMesh->setSurfaceColor(GreedyProjColor);
  greedyMesh->setBackFaceColor(GreedyProjBackFaceColor);
  greedyMesh->setBackFacePolicy(GreedyProjBackFacePolicy);
  greedyMesh->setMaterial(GreedyProjMaterial);
  greedyMesh->setEnabled(enabled);
  greedyMesh->addFaceColorQuantity(GreedyProjFaceColorName, meshFC)->setEnabled(false);

  // Output results
  std::cout << "\nFinished Greedy Projection Triangulation!"  << std::endl;
  std::cout << "\tVertex num\t->\t" << meshV.size()           << std::endl;
  std::cout << "\tFace num\t->\t"   << meshF.size()           << std::endl;
  std::cout                                                   << std::endl;

  return boundaryVerticesIdx;
}

// Show hexagons for each vertex as a pseudo surface.
//  - name: The name of the registered surface
//  - averageDistance:  the radius of the shown hexagon.
//  - enabled:  If true, enable the registered pseudo surface
//  - boundaryVerticesIdx: The indices of the vertices on the hole boundary.
void Surface::renderPseudoSurface(
  std::string name,
  double averageDistance,
  bool enabled,
  const std::set<int> &boundaryVerticesIdx
) {
  size_t N = Vertices->size();
  
  std::vector<glm::dvec3> meshV(N*7);           // Mesh Vertices
  std::vector<std::vector<size_t>> meshF(N*6);  // Mesh Faces
  std::vector<glm::dvec3> meshFC(N*6, PseudoSurfaceColor); // Mesh Face Colors

  // Calculate rotation matrix
  double angleInDegrees = 60.0;
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

    // Register face color
    //  - If the vertex-i is on the hole boundaries, color it red.
    if (boundaryVerticesIdx.count(i)) {
      for (size_t j = 0; j < 6; j++) {
        meshFC[i*6 + j] = PseudoSurfaceBoundaryColor;
      }
    }
  }

  // Register mesh
  polyscope::SurfaceMesh *pseudoSurface = polyscope::registerSurfaceMesh(name, meshV, meshF);
  pseudoSurface->setSurfaceColor(PseudoSurfaceColor);
  pseudoSurface->setBackFaceColor(PseudoSurfaceBackFaceColor);
  pseudoSurface->setMaterial(PseudoSurfaceMaterial);
  pseudoSurface->setBackFacePolicy(PseudoSurfaceBackFacePolicy);
  pseudoSurface->setEnabled(enabled);
  pseudoSurface->addFaceColorQuantity(PseudoSurfaceFaceColorName, meshFC)->setEnabled(false);
}

// Detect the holes on the mesh
//  1.  Detect the edges not shared by two faces
//  2.  Manage vertices of the detected edges with UnionFind
//  3.  Calculate the total length of the hole, and skip 
//      if the length of the hole boundary is less than 
//      the threshold.
//  4.  Return the boundary faces' indices
//
//  - meshV: Vertices on the mesh
//  - meshF: Faces on the mesh
//  - boundaryLengthLimit: Skip if the boundary length is less than this value
std::set<int> Surface::detectHolesOnMesh(
  std::vector<glm::dvec3> &meshV,
  std::vector<std::vector<size_t>> &meshF,
  double boundaryLengthLimit
) {
  const int verticesSize = meshV.size();
  const int facesSize = meshF.size();

  // Detect the edges not shared by two faces
  std::map<int, std::vector<std::pair<int, int>>> vertexToAdjacentEdges;
  std::map<std::pair<int, int>, std::vector<int>> edgeToAdjacentFaces;
  for (int i = 0; i < facesSize; i++) {
    const int polySize = meshF[i].size();
    for (int j = 0; j < polySize; j++) {
      int u = meshF[i][j];
      int v = meshF[i][(j + 1)%polySize];
      std::pair<int, int> e = {u, v};
      if (u >= v) std::swap(e.first, e.second);

      // vertex -> edges
      vertexToAdjacentEdges[u].push_back(e);
      // edge -> faces
      edgeToAdjacentFaces[e].push_back(i);
    }
  }

  // Compute boundary vertices and edges
  std::set<int> boundaryVertices;
  std::set<std::pair<int, int>> boundaryEdges;
  for (auto i: edgeToAdjacentFaces) {
    std::pair<int, int> edge = i.first;
    std::vector<int> facesShareEdge = i.second;
    if (facesShareEdge.size() != 1) continue;

    // Add boundary information
    boundaryVertices.insert(edge.first);
    boundaryVertices.insert(edge.second);
    boundaryEdges.insert(edge);
  }

  // Manage vertices of the detected edges with UnionFind
  UnionFind unionFind(verticesSize);
  for (std::pair<int, int> e: boundaryEdges) {
    unionFind.unite(e.first, e.second);
  }
  
  // Compute root-to-group relationships
  std::map<int, std::set<int>> rootToGroup;
  for (int i = 0; i < verticesSize; i++) {
    if (boundaryVertices.count(i) == 0) continue;

    int root = unionFind.root(i);
    rootToGroup[root].insert(i);
  }

  // Calculate the total length of the hole 
  std::map<int, double> rootToBoundaryLength;
  for (std::pair<int, int> e: boundaryEdges) {
    int root = unionFind.root(e.first);
    double length = glm::length(meshV[e.first] - meshV[e.second]);
    rootToBoundaryLength[root] += length;
  }

  // Return the boundary faces' indices
  std::set<int> holeBoundaryFacesIdx;
  for (std::pair<int, double> i: rootToBoundaryLength) {
    int root = i.first;
    double length = i.second;

    // Skip if the length of the hole boundary is less than the threshold.
    if (length < boundaryLengthLimit) continue;

    for (int vertexIdx: rootToGroup[root]) {
      for (std::pair<int, int> edgeIdx: vertexToAdjacentEdges[vertexIdx]) {
        for (int faceIdx: edgeToAdjacentFaces[edgeIdx]) {
          holeBoundaryFacesIdx.insert(faceIdx);
        }
      }
    }
  }

  return holeBoundaryFacesIdx;
}