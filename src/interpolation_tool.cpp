#include "polyscope/polyscope.h"

#include "polyscope/point_cloud.h"
#include "polyscope/view.h"

#include <map>

#include "constants.hpp"
#include "interpolation_tool.hpp"
#include "surface.hpp"
#include "cluster.hpp"

void InterpolationTool::launchToolOperation() {
  polyscope::view::moveScale = 0.0;

  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    draggingEvent();
  } else {
    releasedEvent();
  }
}

void InterpolationTool::draggingEvent() {
  ImGuiIO &io = ImGui::GetIO();
  ImVec2 mousePos = ImGui::GetMousePos();
  double xPos = io.DisplayFramebufferScale.x * mousePos.x;
  double yPos = io.DisplayFramebufferScale.y * mousePos.y;

  // Cast a ray to screen
  Ray ray(xPos, yPos);
  Ray::Hit hitInfo = ray.castPointToPlane(getScreen());
  assert(hitInfo.hit);
  addSketchPoint(hitInfo.pos);

  // Update surfacePointsIndex
  updateSurfacePoints(xPos, yPos, SurfacePointNum);
  removePointCloud(SurfacePointName);
  registerSurfacePointsAsPointCloud(SurfacePointName);

  // Register sketchPoints as curve network (LINE)
  registerSketchPointsAsCurveNetworkLine(SketchPrefix);
}

void InterpolationTool::releasedEvent() {
  if (getSketchPoints()->size() == 0) return;

  // Find basis points for the surface reconstruction.
  findBasisPoints();
  if (getBasisPointsIndex()->size() == 0) {
    std::cout << "WARNING: No basis point was found." << std::endl;
    removeCurveNetworkLine(SketchPrefix);
    resetSketch();
    return;
  }

  // Register calculated points.
  registerBasisPointsAsPointCloud("Basis Points");

  // Calculate approximate surface with Poisson Surface Reconstruction.
  std::vector<glm::dvec3> basisPoints;
  std::vector<glm::dvec3> basisNormals;
  std::vector<std::vector<size_t>> basisFaces;

  std::set<int>           *basisPointsIndexPtr = getBasisPointsIndex();
  std::vector<glm::dvec3> *verticesPtr = getPointCloud()->getVertices();
  std::vector<glm::dvec3> *normalsPtr = getPointCloud()->getNormals();
  for (int idx: *basisPointsIndexPtr) {
    basisPoints.push_back((*verticesPtr)[idx]);
    basisNormals.push_back((*normalsPtr)[idx]);
  }

  // Poisson Surface Reconstruction
  Surface poissonSurface("Interpolation: PSR", &basisPoints, &basisNormals);
  std::tie(basisPoints, basisFaces) = poissonSurface.reconstructPoissonSurface(getPointCloud()->getAverageDistance());
  if (basisPoints.size() == 0) {
    std::cout << "WARNING: No mesh was reconstructed with Poisson Surface Reconstruction." << std::endl;
    removeCurveNetworkLine(SketchPrefix);
    resetSketch();
    return;
  }

  // Filter the reconstructed surface
  std::vector<glm::dvec3> newV, newN;
  std::tie(newV, newN) = filterSurfacePointsWithFaces(basisPoints, basisFaces);
  if (newV.size() == 0) {
    std::cout << "WARNING: No surface point was selected after filtering method." << std::endl;
    removeCurveNetworkLine(SketchPrefix);
    resetSketch();
    return;
  }

  // Add the interpolated points.
  getPointCloud()->addPoints(newV, newN);

  // Register Interpolated Points as point cloud
  renderInterpolatedPoints(newV, newN);

  // Remove surfacePointsIndex
  removePointCloud(SurfacePointName);

  // Remove sketch as curve network (LINE)
  removeCurveNetworkLine(SketchPrefix);
  
  // Reset all member variables.
  resetSketch();
}

// Register new vertices and normals as point cloud
void InterpolationTool::renderInterpolatedPoints(
  std::vector<glm::dvec3> &newV,
  std::vector<glm::dvec3> &newN
) {
  polyscope::PointCloud* interpolatedCloud = polyscope::registerPointCloud("Interpolated Points", newV);
  interpolatedCloud->setPointColor(glm::dvec3(1.0, 0.0, 0.0));
  interpolatedCloud->setPointRadius(BasisPointRadius);
  interpolatedCloud->setEnabled(false);

  polyscope::PointCloudVectorQuantity* interpolatedVectorQuantity = interpolatedCloud->addVectorQuantity(NormalName, newN);
  interpolatedVectorQuantity->setVectorColor(glm::dvec3(1.0, 0.0, 0.0));
  interpolatedVectorQuantity->setVectorLengthScale(NormalLength);
  interpolatedVectorQuantity->setVectorRadius(NormalRadius * 1.1);
  interpolatedVectorQuantity->setEnabled(NormalEnabled);
  interpolatedVectorQuantity->setMaterial(NormalMaterial);
}


// Filter the reconstructed surface
//  1. Cast reconstructed surface and basisPoints onto the screen plane.
//  2. Construct octree for the casted surface points and basisPoints.
//  3. Search for a candidate point for each discretized grid.
//    - Only points that their normals are directed to cameraOrig.
//    - If the candidate point is one of basisPoints, then skip it.
//  4. Detect depth with DBSCAN
std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> InterpolationTool::filterSurfacePointsWithFaces(
  std::vector<glm::dvec3> &surfacePoints,
  std::vector<std::vector<size_t>> &surfaceFaces
) {
  const int surfacePointsSize = surfacePoints.size();
  const int surfaceFacesSize = surfaceFaces.size();

  // Preprocessing: Calculate the points' average normals 
  std::map<int, glm::dvec3> indexToNormalSum;
  std::map<int, int>        indexToAdjacentCount;
  // Normals of surfacePoints
  for (int i = 0; i < surfaceFacesSize; i++) {
    // Calculate the normal of the triangle
    glm::dvec3 u = surfacePoints[surfaceFaces[i][0]];
    glm::dvec3 v = surfacePoints[surfaceFaces[i][1]];
    glm::dvec3 w = surfacePoints[surfaceFaces[i][2]];

    glm::dvec3 n = glm::normalize(glm::cross(v - u, w - u));
    for (int j = 0; j < 3; j++) {
      int vertexIdx = surfaceFaces[i][j];
      indexToNormalSum[vertexIdx] += n;
      indexToAdjacentCount[vertexIdx]++;
    }
  }

  // Cast reconstructed surface and basisPoints onto the screen plane.
  std::vector<glm::dvec3> pointsInWorldCoord;
  std::vector<glm::dvec3> normalsInWorldCoord;
  std::vector<glm::dvec3> pointsCastedOntoScreen;
  // Cast surface points onto screen
  for (int i = 0; i < surfacePointsSize; i++) {
    glm::dvec3 p = surfacePoints[i];
    glm::dvec3 pn = indexToNormalSum[i]/(double)indexToAdjacentCount[i];
    
    // Cast a ray from p to cameraOrig onto screen.
    Ray ray(p, getCameraOrig());
    Ray::Hit hitInfo = ray.castPointToPlane(getScreen());
    assert(hitInfo.hit);

    pointsInWorldCoord.push_back(p);
    normalsInWorldCoord.push_back(pn);
    pointsCastedOntoScreen.push_back(getScreen()->mapCoordinates(hitInfo.pos));
  }
  
  // Cast basis points onto screen
  std::set<int>           *basisPointsIndexPtr = getBasisPointsIndex();
  std::vector<glm::dvec3> *verticesPtr = getPointCloud()->getVertices();
  std::vector<glm::dvec3> *normalsPtr = getPointCloud()->getNormals();
  for (int idx: *basisPointsIndexPtr) {
    glm::dvec3 p = (*verticesPtr)[idx];
    glm::dvec3 pn = (*normalsPtr)[idx];

    // Cast a ray from p to cameraOrig onto screen.
    Ray ray(p, getCameraOrig());
    Ray::Hit hitInfo = ray.castPointToPlane(getScreen());
    assert(hitInfo.hit);

    pointsInWorldCoord.push_back(p);
    normalsInWorldCoord.push_back(pn);
    pointsCastedOntoScreen.push_back(getScreen()->mapCoordinates(hitInfo.pos));
  }

  return filterSurfacePointsInner(
    surfacePointsSize,
    pointsInWorldCoord,
    normalsInWorldCoord,
    pointsCastedOntoScreen
  );
}

std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> InterpolationTool::filterSurfacePointsWithNormals(
  std::vector<glm::dvec3> &surfacePoints,
  std::vector<glm::dvec3> &surfaceNormals
) {
  const int surfacePointsSize = surfacePoints.size();
  const int surfaceNormalsSize = surfaceNormals.size();

  assert(surfacePointsSize == surfaceNormalsSize);

  // Cast reconstructed surface and basisPoints onto the screen plane.
  std::vector<glm::dvec3> pointsInWorldCoord;
  std::vector<glm::dvec3> normalsInWorldCoord;
  std::vector<glm::dvec3> pointsCastedOntoScreen;
  // Cast surface points onto screen
  for (int i = 0; i < surfacePointsSize; i++) {
    glm::dvec3 p = surfacePoints[i];
    glm::dvec3 pn = surfaceNormals[i];

    // Cast a ray from p to cameraOrig onto screen.
    Ray ray(p, getCameraOrig());
    Ray::Hit hitInfo = ray.castPointToPlane(getScreen());
    assert(hitInfo.hit);

    pointsInWorldCoord.push_back(p);
    normalsInWorldCoord.push_back(pn);
    pointsCastedOntoScreen.push_back(getScreen()->mapCoordinates(hitInfo.pos));
  }

  // Cast basis points onto screen
  std::set<int>           *basisPointsIndexPtr = getBasisPointsIndex();
  std::vector<glm::dvec3> *verticesPtr = getPointCloud()->getVertices();
  std::vector<glm::dvec3> *normalsPtr = getPointCloud()->getNormals();
  for (int idx: *basisPointsIndexPtr) {
    glm::dvec3 p = (*verticesPtr)[idx];
    glm::dvec3 pn = (*normalsPtr)[idx];

    // Cast a ray from p to cameraOrig onto screen.
    Ray ray(p, getCameraOrig());
    Ray::Hit hitInfo = ray.castPointToPlane(getScreen());
    assert(hitInfo.hit);

    pointsInWorldCoord.push_back(p);
    normalsInWorldCoord.push_back(pn);
    pointsCastedOntoScreen.push_back(getScreen()->mapCoordinates(hitInfo.pos));
  }

  return filterSurfacePointsInner(
    surfacePointsSize,
    pointsInWorldCoord,
    normalsInWorldCoord,
    pointsCastedOntoScreen
  );
}

std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> InterpolationTool::filterSurfacePointsInner(
  int surfacePointsSize,
  std::vector<glm::dvec3> &pointsInWorldCoord,
  std::vector<glm::dvec3> &normalsInWorldCoord,
  std::vector<glm::dvec3> &pointsCastedOntoScreen
) {
  const int pointSize = pointsInWorldCoord.size();

  // Construct octree for the casted surface points and basis Points
  pcl::PointCloud<pcl::PointXYZ>::Ptr castedSurfaceCloud(new pcl::PointCloud<pcl::PointXYZ>);
  castedSurfaceCloud->points.resize(pointSize);
  for (int i = 0; i < pointSize; i++) {
    castedSurfaceCloud->points[i].x = pointsCastedOntoScreen[i].x;
    castedSurfaceCloud->points[i].y = pointsCastedOntoScreen[i].y;
    castedSurfaceCloud->points[i].z = pointsCastedOntoScreen[i].z;
  }
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(OctreeResolution);
  octree.setInputCloud(castedSurfaceCloud);
  octree.addPointsFromInputCloud();

  // Search for a candidate point for each discretized grid.
  const double INF = 1e5;
  double min_x = INF, max_x = -INF;
  double min_y = INF, max_y = -INF;
  for (int i = 0; i < pointSize; i++) {
    glm::dvec3 mapped_p = pointsCastedOntoScreen[i];

    min_x = std::min(min_x, mapped_p.x);
    max_x = std::max(max_x, mapped_p.x);
    min_y = std::min(min_y, mapped_p.y);
    max_y = std::max(max_y, mapped_p.y);
  }

  const double castedAverageDist = calcCastedAverageDist();
  std::set<int> candidatePointsIndexSet;
  std::vector<int> candidatePointsIndex;
  std::vector<glm::dvec3> hasCloserNeighborPoints;
  for (double x = min_x; x < max_x; x += castedAverageDist) {
    for (double y = min_y; y < max_y; y += castedAverageDist) {
      if (!insideSketch(x, y)) continue;
      if (!insideBasisConvexHull(x, y)) continue;

      // Search for nearest neighbors with octree
      std::vector<int>    hitPointIndices;
      std::vector<float>  hitPointDistances;
      int hitPointCount = octree.radiusSearch(
        pcl::PointXYZ(x, y, 0.0),
        castedAverageDist,
        hitPointIndices,
        hitPointDistances
      );

      // Get the index of the min depth point 
      double minDepth = 1e5;
      int minDepthIdx = -1;
      for (int i = 0; i < hitPointCount; i++) {
        int hitPointIdx = hitPointIndices[i];
        glm::dvec3 p = pointsInWorldCoord[hitPointIdx];
        glm::dvec3 pn = normalsInWorldCoord[hitPointIdx];

        double curDepth = glm::length(p - getCameraOrig());
        if (curDepth < minDepth) {
          minDepth = curDepth;
          minDepthIdx = hitPointIdx;
        }
      }

      // Discard points that is one of basisPoints.
      if (minDepthIdx >= surfacePointsSize) continue;

      // If the point is already selected, then skip it.
      if (candidatePointsIndexSet.count(minDepthIdx)) continue;

      // Update the buffer vectors
      for (int i = 0; i < hitPointCount; i++) {
        int hitPointIdx = hitPointIndices[i];
        glm::dvec3 p = pointsInWorldCoord[hitPointIdx];
        glm::dvec3 pn = normalsInWorldCoord[hitPointIdx];

        // Discard points that their normals are not directed to cameraOrig.
        if (glm::dot(pn, p - getCameraOrig()) >= 0.0) continue;

        if (hitPointIdx == minDepthIdx) {
          candidatePointsIndexSet.insert(hitPointIdx);
          candidatePointsIndex.push_back(hitPointIdx);
        } else {
          hasCloserNeighborPoints.push_back(p);
        }
      }
    }
  }

  // Register points that has closer nearest neighbors as point cloud.
  polyscope::PointCloud* hasCloserNeighborCloud = polyscope::registerPointCloud("HasCloserNeighbor(surface)", hasCloserNeighborPoints);
  hasCloserNeighborCloud->setPointRadius(BasisPointRadius);
  hasCloserNeighborCloud->setEnabled(false);

  // Detect depth with DBSCAN
  Clustering clustering(&candidatePointsIndex, &pointsInWorldCoord, "surface");
  std::set<int> clusteredPointsIndex = clustering.executeClustering(
    DBSCAN_SearchRange*getPointCloud()->getAverageDistance(),
    DBSCAN_MinPoints,
    CLUSTER_MIN_DEPTH
  );

  // Filter the added vertices in order to preserve the density of the point cloud
  std::vector<glm::dvec3> clusteredV;
  std::vector<glm::dvec3> clusteredN;
  for (int idx: clusteredPointsIndex) {
    glm::dvec3 p = pointsInWorldCoord[idx];
    glm::dvec3 pn = normalsInWorldCoord[idx];

    clusteredV.push_back(p);
    clusteredN.push_back(pn);
  }
  std::vector<int> filteredIndex = voxelFilter(*(getPointCloud()->getVertices()), clusteredV);
  std::vector<glm::dvec3> newV(filteredIndex.size());
  std::vector<glm::dvec3> newN(filteredIndex.size());
  for (size_t i = 0; i < filteredIndex.size(); i++) {
    int idx = filteredIndex[i];
    newV[i] = clusteredV[idx];
    newN[i] = clusteredN[idx];
  }

  return { newV, newN };
}