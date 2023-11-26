#include "polyscope/polyscope.h"

#include "polyscope/point_cloud.h"
#include "polyscope/view.h"

#include <map>

#include "constants.hpp"
#include "interpolation_tool.hpp"
#include "rbf.hpp"
#include "surface.hpp"

bool InterpolationTool::drawSketch() {
  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    draggingEvent();
    return false;
  } else if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && getSketchPoints()->size() > 0) {
    releasedEvent();
    return true;
  }

  return false;
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
  // Find basis points.
  //  - If extendedSearch is true, extend the sketched area.
  //  - Cast all points of the point cloud onto the screen plane.
  //  - Check the conditions below.
  //    1. Judge inside/outside of the sketch.
  //    2. Check the normal direction of the point.
  //    3. Check the nearest neighbors' distances from cameraOrig.
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
  std::tie(psrPoints, psrFaces) = poissonReconstruct(
    "Interpolation: PSR",
    getPointCloud()->getAverageDistance(),
    psrPoints,
    psrNormals  
  );
  if (psrPoints.rows() == 0) {
    std::cout << "WARNING: No mesh was reconstructed with Poisson Surface Reconstruction." << std::endl;
    removeCurveNetworkLine(SketchPrefix);
    resetSketch();
    return;
  }

  // Filter the reconstructed surface
  //  - Cast reconstructed surface onto the screen plane.
  //  - Filter only the points inside of the sketch.
  //  - Uniform the density of interpolated points.
  Eigen::MatrixXd newV, newN;
  std::tie(newV, newN) = filterSurfacePoints(psrPoints, psrFaces);
  if (newV.rows() == 0) {
    std::cout << "WARNING: No surface point was selected after filtering method." << std::endl;
    removeCurveNetworkLine(SketchPrefix);
    resetSketch();
    return;
  }

  // Add the interpolated points.
  getPointCloud()->addPoints(newV, newN);

  // Register Interpolated Points as point cloud
  renderInterpolatedPoints(newV, newN);

  // Remove sketch as curve network (LINE)
  removeCurveNetworkLine(SketchPrefix);

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
  interpolatedCloud->setEnabled(BasisPointEnabled);

  polyscope::PointCloudVectorQuantity* interpolatedVectorQuantity = interpolatedCloud->addVectorQuantity(NormalName, newN);
  interpolatedVectorQuantity->setVectorColor(glm::dvec3(1.0, 0.0, 0.0));
  interpolatedVectorQuantity->setVectorLengthScale(NormalLength);
  interpolatedVectorQuantity->setVectorRadius(NormalRadius * 1.1);
  interpolatedVectorQuantity->setEnabled(NormalEnabled);
  interpolatedVectorQuantity->setMaterial(NormalMaterial);
}

// Filter the reconstructed surface
//  - Cast reconstructed surface onto the screen plane.
//  - Filter only the points inside of the sketch and the convex hull of basisPoints.
//  - Uniform the density of interpolated points.
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> InterpolationTool::filterSurfacePoints(
  Eigen::MatrixXd &surfacePoints,
  Eigen::MatrixXi &surfaceFaces
) {
  // Preprocessing: Calculate the points' average normals 
  std::map<int, glm::dvec3> indexToNormalSum;
  std::map<int, int>        indexToAdjacentCount;
  for (int i = 0; i < surfaceFaces.rows(); i++) {
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

  // Cast reconstructed surface onto the screen plane
  std::vector<glm::dvec3> pointsCastedOntoScreen;
  for (int i = 0; i < surfacePoints.rows(); i++) {
    glm::dvec3 p = glm::dvec3(
      surfacePoints(i, 0),
      surfacePoints(i, 1),
      surfacePoints(i, 2)
    );
    
    // Cast a ray from p to cameraOrig onto screen.
    Ray ray(p, getCameraOrig());
    Hit hitInfo = ray.castPointToPlane(getScreen());
    if (hitInfo.hit) {
      glm::dvec3 castedP = getScreen()->mapCoordinates(hitInfo.pos);
      pointsCastedOntoScreen.push_back(castedP);
    }
  }

  // Filter only the points inside of the sketch and the convex hull of basisPoints
  std::vector<int> insideSketchPointIndex;
  for (int i = 0; i < pointsCastedOntoScreen.size(); i++) {
    glm::dvec3 p = pointsCastedOntoScreen[i];

    if (insideSketch(p.x, p.y) && insideBasisConvexHull(p.x, p.y)) {
      insideSketchPointIndex.push_back(i);
    }
  }

  // Uniform the density of interpolated points.
  // TODO: Implement here later...

  // Register the vertex and the normal,
  // The normal and the camera direction must be faced each other.
  std::vector<int> insideSketchPointIndexBuffer;
  for (int i = 0; i < insideSketchPointIndex.size(); i++) {
    int idx = insideSketchPointIndex[i];
    glm::dvec3 normal = indexToNormalSum[idx]/(double)indexToAdjacentCount[idx];
    if (glm::dot(getCameraDir(), normal) < 0.0) insideSketchPointIndexBuffer.push_back(idx);
  }
  insideSketchPointIndex = insideSketchPointIndexBuffer;
  
  int newPointsSize = insideSketchPointIndex.size();
  Eigen::MatrixXd newV(newPointsSize, 3);
  Eigen::MatrixXd newN(newPointsSize, 3);
  for (int i = 0; i < newPointsSize; i++) {
    int idx = insideSketchPointIndex[i];
    glm::dvec3 normal = indexToNormalSum[idx]/(double)indexToAdjacentCount[idx];

    newV.row(i) <<
      surfacePoints(idx, 0),
      surfacePoints(idx, 1),
      surfacePoints(idx, 2);
    newN.row(i) << 
      normal.x,
      normal.y,
      normal.z;
  }

  return { newV, newN };
}