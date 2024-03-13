#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"

#include <igl/readOBJ.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <string>
#include <fstream>

#include "point_cloud.hpp"
#include "constants.hpp"
#include "surface.hpp"

PointCloud::PointCloud(std::string filename, bool downsample) 
: inputCloud(new pcl::PointCloud<pcl::PointXYZ>), octree(OctreeResolution) {
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

  // Scale the input point cloud
  // If downsample flag is set, then execute downsampling
  scalePointCloud();
  if (downsample) {
    std::set<int> donwsampledIndex = downsampling(PointCloudDownsampleVoxel);
    std::vector<glm::dvec3> downsampledPoints, downsampledNormals;
    for (int idx: donwsampledIndex) {
      downsampledPoints.push_back(Vertices[idx]);
      downsampledNormals.push_back(Normals[idx]);
    }
    Vertices = downsampledPoints;
    Normals = downsampledNormals;
  }

  // Update point cloud
  //    - update environments
  //    - update octree
  //    - render points and normals
  updatePointCloud(false);
} 

// Output current Vertices and Normals as .obj file
void PointCloud::exportOBJFile(std::string logFileName) {
  std::string logFileNameOBJ = logFileName + ".obj";
  std::ofstream objFile(logFileNameOBJ);

  if (objFile.is_open()) {
    // Write vertex information to .obj
    for (glm::dvec3 p: Vertices) {
      std::string px = std::to_string(p.x);
      std::string py = std::to_string(p.y);
      std::string pz = std::to_string(p.z);

      objFile << "v " << px << ' ' << py << ' ' << pz << '\n';
    }

    // Write normal information to .obj
    for (glm::dvec3 pn: Normals) {
      std::string pn_x = std::to_string(pn.x);
      std::string pn_y = std::to_string(pn.y);
      std::string pn_z = std::to_string(pn.z);

      objFile << "vn " << pn_x << ' ' << pn_y << ' ' << pn_z << '\n';
    }

    // Close objFile
    objFile.close();
    std::cout << "Exported " << logFileNameOBJ << std::endl;
  } else {
    std::cout << "WARNING: " << logFileNameOBJ << " was not opened." << std::endl;
  }

  return;
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
  setPointCloudUpdated(true);
  prevEnvironments.push({ Vertices, Normals });
  if (clearPostEnv) postEnvironments = std::stack<std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>>>();

  // Update Octree
  updateOctree();

  // Calculate average distance between the nearest points.
  if (averageDistance == 0.0) averageDistance = calcAverageDistance();

  // Clear the buffer data and remove temporal pseudo surface
  VerticesBuffer.clear();
  NormalsBuffer.clear();
  if (polyscope::hasSurfaceMesh(TemporalPseudoSurfaceName)) {
    polyscope::SurfaceMesh *temporalPseudoSurface = polyscope::getSurfaceMesh(TemporalPseudoSurfaceName);
    temporalPseudoSurface->setEnabled(false);
  }

  // Render point cloud surface (pseudo surface and greedy surface)
  Surface pointCloudSurface(&Vertices, &Normals);
  pointCloudSurface.renderPointCloudSurface(
    GreedyProjName,
    PseudoSurfaceName,
    averageDistance,
    false,
    true
  );

  // Register Points
  pointCloud = polyscope::registerPointCloud(PointName, Vertices);
  pointCloud->setPointColor(PointColor);
  pointCloud->setPointRadius(PointRadius);
  pointCloud->setEnabled(false);

  // Register Normals
  vectorQuantity = pointCloud->addVectorQuantity(NormalName, Normals);
  vectorQuantity->setVectorColor(NormalColor);
  vectorQuantity->setVectorLengthScale(NormalLength);
  vectorQuantity->setVectorRadius(NormalRadius);
  vectorQuantity->setEnabled(NormalEnabled);
  vectorQuantity->setMaterial(NormalMaterial);

  std::cout << "Point Cloud Data:"                                      << std::endl;
  std::cout << "\tVertex num\t\t->\t"           << Vertices.size()      << std::endl;
  std::cout << "\tNormal num\t\t->\t"           << Normals.size()       << std::endl;
  std::cout << "\tAverage Distance\t->\t"       << averageDistance      << std::endl;
  std::cout << "\tBoundng Box Side\t->\t"       << boundingBoxSide      << std::endl;
  std::cout                                                             << std::endl;
}

// Add vertices from the positions and normals
void PointCloud::addPoints(std::vector<glm::dvec3> &newV, std::vector<glm::dvec3> &newN) {
  assert(newV.size() == newN.size());
  const int addedSize = newV.size();

  // Add new points to octree
  for (int i = 0; i < addedSize; i++) {
    glm::dvec3 p = newV[i];
    octree.addPointToCloud(pcl::PointXYZ(p.x, p.y, p.z), inputCloud);
  }

  // Add new points to VerticesBuffer and NormalsBuffer
  for (int i = 0; i < addedSize; i++) {
    Vertices.push_back(newV[i]);
    Normals.push_back(newN[i]);
  }

  // Add new points to VerticesBuffer and NormalsBuffer
  for (int i = 0; i < addedSize; i++) {
    VerticesBuffer.push_back(newV[i]);
    NormalsBuffer.push_back(newN[i]);
  }

  // Render added points temporarily
  Surface pseudoSurfaceTemporal(&VerticesBuffer, &NormalsBuffer);
  pseudoSurfaceTemporal.renderPseudoSurface(
    TemporalPseudoSurfaceName,
    averageDistance, 
    true
  );
}

// Delete vertices by referencing the vertex indices
void PointCloud::deletePoints(std::set<int> &indices) {
  if (indices.size() == 0) return;

  const int curSize = Vertices.size();
  const int newSize = Vertices.size() - indices.size();

  std::vector<glm::dvec3> newV;
  std::vector<glm::dvec3> newN;

  std::set<int>::iterator itr = indices.begin();
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

// Get values of member variables
bool PointCloud::getPointCloudUpdated() {
  return pointCloudUpdated;
}
std::vector<glm::dvec3>* PointCloud::getVertices() {
  return &Vertices;
}
std::vector<glm::dvec3>* PointCloud::getNormals() {
  return &Normals;
}
double PointCloud::getAverageDistance() {
  return averageDistance;
}
double PointCloud::getBoundingBoxSide() {
  return boundingBoxSide;
}
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* PointCloud::getOctree() {
  return &octree;
}

// Set values to member variables
void PointCloud::setPointCloudUpdated(bool newval) {
  pointCloudUpdated = newval;
}

// Move points to set the gravity point to (0.0, 0.0, 0.0),
// and then scale the point cloud so that the bounding box side is 1.0
void PointCloud::scalePointCloud() {
  glm::dvec3 center(0.0);

  // Move points to set the gravity point to (0.0, 0.0, 0.0),
  for (size_t i = 0; i < Vertices.size(); i++) {
    center += Vertices[i];
  }
  center /= static_cast<double>(Vertices.size());

  // Scale the point cloud so that the bounding box side is 1.0
  boundingBoxSide = 0.0;
  for (size_t i = 0; i < Vertices.size(); i++) {
    Vertices[i] -= center;

    boundingBoxSide = std::max(boundingBoxSide, std::abs(Vertices[i].x));
    boundingBoxSide = std::max(boundingBoxSide, std::abs(Vertices[i].y));
    boundingBoxSide = std::max(boundingBoxSide, std::abs(Vertices[i].z));
  }
  for (size_t i = 0; i < Vertices.size(); i++) {
    // The bounding box is centered at the origin,
    // so it is normalized by multiplying by 0.t
    Vertices[i] *= (PointCloudBoundingBoxSide/2.0) / boundingBoxSide;
  }
  boundingBoxSide = PointCloudBoundingBoxSide;
}

// Filter Vertices by selecting the candidate
// point for each voxel.
//  - voxelSide:      the length of the voxel side
std::set<int> PointCloud::downsampling(double voxelSide) {
  // map for voxel filter
  //  - std::tupple<double, double, double>:  the index of the voxel
  //  - std::pair<double, int>: the distance between the center point and the current 
  //                            candidate point and the index of the current candidate point
  std::map<std::tuple<int, int, int>, std::pair<double, int>> voxels;

  // Compute whether each vertex is a candidate point.
  std::vector<bool> isCandidatePoint(Vertices.size());
  for (size_t i = 0; i < Vertices.size(); i++) {
    glm::dvec3 p = Vertices[i];
    std::tuple<int, int, int> idx = {
      std::floor(p.x/voxelSide),
      std::floor(p.y/voxelSide),
      std::floor(p.z/voxelSide)
    };

    std::pair<double, int> currentCandidate = { 1e5, i };
    if (voxels.count(idx) != 0) {
      currentCandidate = voxels[idx];
    }

    double currentCandidateDist = currentCandidate.first;
    int currentCandidateIdx = currentCandidate.second;

    // Update the voxels information
    glm::dvec3 voxelBasis = glm::dvec3(
      (double)std::get<0>(idx)*voxelSide,
      (double)std::get<1>(idx)*voxelSide,
      (double)std::get<2>(idx)*voxelSide
    );
    glm::dvec3 voxelCenter = voxelBasis + 0.5*glm::dvec3(voxelSide, voxelSide, voxelSide);
    double currentDist = glm::length(p - voxelCenter);
    if (currentDist < currentCandidateDist) {
      voxels[idx] = { currentDist, i };
      isCandidatePoint[currentCandidateIdx] = false;
      isCandidatePoint[i] = true;
    }
  }

  std::set<int> filteredIndex;
  for (size_t i = 0; i < Vertices.size(); i++) {
    if (isCandidatePoint[i]) filteredIndex.insert(i);
  }

  return filteredIndex;
}

// Update registered vertices
void PointCloud::updateOctree() {
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
