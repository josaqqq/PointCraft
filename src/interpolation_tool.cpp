#include "polyscope/polyscope.h"

#include "polyscope/point_cloud.h"
#include "polyscope/view.h"

#include <map>

#include "constants.hpp"
#include "interpolation_tool.hpp"
#include "rbf.hpp"
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

  // Cast a ray
  Ray ray(xPos, yPos);
  Hit hitInfo = ray.castPointToPlane(getScreen());
  if (hitInfo.hit) addSketchPoint(hitInfo.pos);

  // Register sketchPoints as curve network (LINE)
  registerSketchPointsAsCurveNetworkLine(SketchPrefix);
}

void InterpolationTool::releasedEvent() {
  if (getSketchPoints()->size() == 0) return;

  // Find basis points for the surface reconstruction.
  findBasisPoints(true);
  if (getBasisPointsIndex()->size() == 0) {
    std::cout << "WARNING: No basis point was found." << std::endl;
    removeCurveNetworkLine(SketchPrefix);
    resetSketch();
    return;
  }

  // Register calculated points.
  registerBasisPointsAsPointCloud("Basis Points");

  // Calculate approximate surface with Poisson Surface Reconstruction.
  int basisPointsSize = getBasisPointsIndex()->size();
  Eigen::MatrixXd psrPoints(basisPointsSize, 3);
  Eigen::MatrixXd psrNormals(basisPointsSize, 3);
  Eigen::MatrixXi psrFaces;
  for (int i = 0; i < basisPointsSize; i++) {
    int idx = (*getBasisPointsIndex())[i];
    psrPoints(i, 0) = getPointCloud()->Vertices(idx, 0);
    psrPoints(i, 1) = getPointCloud()->Vertices(idx, 1);
    psrPoints(i, 2) = getPointCloud()->Vertices(idx, 2);

    psrNormals(i, 0) = getPointCloud()->Normals(idx, 0);
    psrNormals(i, 1) = getPointCloud()->Normals(idx, 1);
    psrNormals(i, 2) = getPointCloud()->Normals(idx, 2);
  }
  Surface poissonSurface("Interpolation: PSR", &psrPoints, &psrNormals);
  std::tie(psrPoints, psrFaces) = poissonSurface.reconstructPoissonSurface(getPointCloud()->getAverageDistance());
  if (psrPoints.rows() == 0) {
    std::cout << "WARNING: No mesh was reconstructed with Poisson Surface Reconstruction." << std::endl;
    removeCurveNetworkLine(SketchPrefix);
    resetSketch();
    return;
  }

  // Filter the reconstructed surface
  Eigen::MatrixXd newV, newN;
  std::tie(newV, newN) = filterSurfacePoints(psrPoints, psrFaces);
  if (newV.rows() == 0) {
    std::cout << "WARNING: No surface point was selected after filtering method." << std::endl;
    removeCurveNetworkLine(SketchPrefix);
    resetSketch();
    return;
  }

  // Register Interpolated Points as point cloud
  renderInterpolatedPoints(newV, newN);

  // Remove sketch as curve network (LINE)
  removeCurveNetworkLine(SketchPrefix);

  // Add the interpolated points.
  getPointCloud()->addPoints(newV, newN);
  
  // Reset all member variables.
  resetSketch();
}

// Register new vertices and normals as point cloud
void InterpolationTool::renderInterpolatedPoints(
  Eigen::MatrixXd &newV, 
  Eigen::MatrixXd &newN
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
//  - Cast reconstructed surface and basisPoints onto the screen plane.
//  - Construct octree for the casted surface points and basisPoints.
//  - Search for a candidate point for each discretized grid.
//    - Only points that their normals are directed to cameraOrig.
//    - If the candidate point is one of basisPoints, then skip it.
//  - Detect depth with DBSCAN
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> InterpolationTool::filterSurfacePoints(
  Eigen::MatrixXd &surfacePoints,
  Eigen::MatrixXi &surfaceFaces
) {
  const int surfacePointsSize = surfacePoints.rows();
  const int surfaceFacesSize = surfaceFaces.rows();
  const int basisPointsSize = getBasisPointsIndex()->size();

  // Preprocessing: Calculate the points' average normals 
  std::map<int, glm::dvec3> indexToNormalSum;
  std::map<int, int>        indexToAdjacentCount;
  // Normals of surfacePoints
  for (int i = 0; i < surfaceFacesSize; i++) {
    // Calculate the normal of the triangle
    glm::dvec3 u = glm::dvec3(
      surfacePoints(surfaceFaces(i, 0), 0), 
      surfacePoints(surfaceFaces(i, 0), 1), 
      surfacePoints(surfaceFaces(i, 0), 2)
    );
    glm::dvec3 v = glm::dvec3(
      surfacePoints(surfaceFaces(i, 1), 0), 
      surfacePoints(surfaceFaces(i, 1), 1), 
      surfacePoints(surfaceFaces(i, 1), 2)
    );
    glm::dvec3 w = glm::dvec3(
      surfacePoints(surfaceFaces(i, 2), 0), 
      surfacePoints(surfaceFaces(i, 2), 1), 
      surfacePoints(surfaceFaces(i, 2), 2)
    );

    glm::dvec3 n = glm::normalize(glm::cross(v - u, w - u));
    for (int j = 0; j < 3; j++) {
      int vertexIdx = surfaceFaces(i, j);
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
    glm::dvec3 p = glm::dvec3(
      surfacePoints(i, 0),
      surfacePoints(i, 1),
      surfacePoints(i, 2)
    );
    glm::dvec3 pn = indexToNormalSum[i]/(double)indexToAdjacentCount[i];
    
    // Cast a ray from p to cameraOrig onto screen.
    Ray ray(p, getCameraOrig());
    Hit hitInfo = ray.castPointToPlane(getScreen());
    assert(hitInfo.hit == true);

    pointsInWorldCoord.push_back(p);
    normalsInWorldCoord.push_back(pn);
    pointsCastedOntoScreen.push_back(getScreen()->mapCoordinates(hitInfo.pos));
  }
  // Cast basis points onto screen
  for (int i = 0; i < basisPointsSize; i++) {
    int pointCloudIdx = (*getBasisPointsIndex())[i];
    glm::dvec3 p = glm::dvec3(
      getPointCloud()->Vertices(pointCloudIdx, 0),
      getPointCloud()->Vertices(pointCloudIdx, 1),
      getPointCloud()->Vertices(pointCloudIdx, 2)
    );
    glm::dvec3 pn = glm::dvec3(
      getPointCloud()->Normals(pointCloudIdx, 0),
      getPointCloud()->Normals(pointCloudIdx, 1),
      getPointCloud()->Normals(pointCloudIdx, 2)
    );

    // Cast a ray from p to cameraOrig onto screen.
    Ray ray(p, getCameraOrig());
    Hit hitInfo = ray.castPointToPlane(getScreen());
    assert(hitInfo.hit == true);

    pointsInWorldCoord.push_back(p);
    normalsInWorldCoord.push_back(pn);
    pointsCastedOntoScreen.push_back(getScreen()->mapCoordinates(hitInfo.pos));
  }

  // Construct octree for the casted surface points and basis Points
  pcl::PointCloud<pcl::PointXYZ>::Ptr castedSurfaceCloud(new pcl::PointCloud<pcl::PointXYZ>);
  int octreeSize = surfacePointsSize + basisPointsSize;
  castedSurfaceCloud->points.resize(octreeSize);
  for (int i = 0; i < octreeSize; i++) {
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
  for (int i = 0; i < octreeSize; i++) {
    glm::dvec3 mapped_p = pointsCastedOntoScreen[i];

    min_x = std::min(min_x, mapped_p.x);
    max_x = std::max(max_x, mapped_p.x);
    min_y = std::min(min_y, mapped_p.y);
    max_y = std::max(max_y, mapped_p.y);
  }

  const double castedAverageDist = calcCastedAverageDist();
  std::vector<int> candidatePointsIndex;
  std::vector<std::vector<double>> hasCloserNeighborPoints;
  for (double x = min_x; x < max_x; x += castedAverageDist*2.0) {
    for (double y = min_y; y < max_y; y += castedAverageDist*2.0) {
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

        // Discard points that their normals are directed to cameraOrig.
        if (glm::dot(pn, getCameraDir()) >= 0.0) continue;

        double curDepth = glm::length(p - getCameraOrig());
        if (curDepth < minDepth) {
          minDepth = curDepth;
          minDepthIdx = hitPointIdx;
        }
      }

      // Discard points that is one of basisPoints.
      if (minDepthIdx >= surfacePointsSize) continue;

      // Update the buffer vectors
      for (int i = 0; i < hitPointCount; i++) {
        int hitPointIdx = hitPointIndices[i];
        if (hitPointIdx == minDepthIdx) {
          candidatePointsIndex.push_back(hitPointIdx);
        } else {
          glm::dvec3 pn = normalsInWorldCoord[hitPointIdx];
          if (glm::dot(pn, getCameraDir()) >= 0.0) continue;

          hasCloserNeighborPoints.push_back({
            pointsInWorldCoord[hitPointIdx].x,
            pointsInWorldCoord[hitPointIdx].y,
            pointsInWorldCoord[hitPointIdx].z
          });
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
  std::vector<int> interpolatedPointsIndex = clustering.executeClustering(
    DBSCAN_SearchRange*getPointCloud()->getAverageDistance(),
    DBSCAN_MinPoints,
    CLUSTER_MIN_DEPTH
  );

  int newPointsSize = interpolatedPointsIndex.size();
  Eigen::MatrixXd newV(newPointsSize, 3);
  Eigen::MatrixXd newN(newPointsSize, 3);
  for (int i = 0; i < newPointsSize; i++) {
    int idx = interpolatedPointsIndex[i];
    glm::dvec3 p = pointsInWorldCoord[idx];
    glm::dvec3 pn = normalsInWorldCoord[idx];

    newV.row(i) << p.x, p.y, p.z;
    newN.row(i) << pn.x, pn.y, pn.z;
  }

  return { newV, newN };
}