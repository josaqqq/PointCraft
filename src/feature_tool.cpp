#include "polyscope/view.h"

#include "feature_tool.hpp"
#include "surface.hpp"
#include "constants.hpp"

void FeatureTool::launchToolOperation() {
  polyscope::view::moveScale = 0.0;

  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    draggingEvent();
  } else {
    releasedEvent();
  }
}

void FeatureTool::draggingEvent() {
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
  updateSurfacePoints(xPos, yPos, *getSurfacePointNumPtr());
  disablePointCloud(SurfacePointName);
  registerSurfacePoints(SurfacePointName);

  // Register:
  //  - sketch
  registerSketch(SketchPrefix);
}

void FeatureTool::releasedEvent() {
  if (getSketchPoints()->size() == 0) return;

  // Find basis points for the surface reconstruction.
  findBasisPoints(true, CLUSTER_MAX_SIZE);
  if (getBasisPointsIndex()->size() == 0) {
    // Disable:
    //  - surface points
    //  - sketch
    disablePointCloud(SurfacePointName);
    disableSketch(SketchPrefix);
    // Reset all member variables.
    resetSketch();
    std::cout << "WARNING: No basis point was found." << std::endl;
    return;
  }

  // Fetch basis points information from the point cloud.
  std::set<int> *basisPointsIndexPtr    = getBasisPointsIndex();
  std::vector<glm::dvec3> *verticesPtr  = getPointCloud()->getVertices();
  std::vector<glm::dvec3> *normalsPtr   = getPointCloud()->getNormals();

  std::vector<glm::dvec3> basisPoints, basisNormals;
  for (int idx: *basisPointsIndexPtr) {
    basisPoints.push_back((*verticesPtr)[idx]);
    basisNormals.push_back((*normalsPtr)[idx]);
  }

  // Compute Poisson Surface Reconstruction
  std::vector<glm::dvec3> poissonPoints;
  std::vector<glm::dvec3> poissonNormals;

  std::string poissonName = "Poisson Interpolation ";
  poissonName += std::to_string(poissonNum);
  poissonNum++;

  Surface poissonSurface(&basisPoints, &basisNormals);
  std::tie(poissonPoints, poissonNormals) = poissonSurface.reconstructPoissonSurface(
    poissonName,
    true
  );
  if (poissonPoints.size() == 0) {
    // Disable:
    //  - surface points
    //  - sketch
    disablePointCloud(SurfacePointName);
    disableSketch(SketchPrefix);
    // Reset all member variables.
    resetSketch();
    std::cout << "WARNING: No mesh was reconstructed with Poisson Surface Reconstruction." << std::endl;
    return;
  }

  // Disable:
  //  - surface points
  //  - sketch
  disablePointCloud(SurfacePointName);
  disableSketch(SketchPrefix);

  // Reset all member variables.
  resetSketch();
}
