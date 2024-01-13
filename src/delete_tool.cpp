#include "polyscope/view.h"

#include "delete_tool.hpp"
#include "cluster.hpp"
#include "constants.hpp"

void DeleteTool::launchToolOperation() {
  polyscope::view::moveScale = 0.0;

  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    draggingEvent();
  } else {
    releasedEvent();
  }
}

void DeleteTool::draggingEvent() {
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

void DeleteTool::releasedEvent() {
  if (getSketchPoints()->size() == 0) return;

  // Find basis points for the surface reconstruction.
  findBasisPoints(false, CLUSTER_MIN_DEPTH);
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

  std::set<int> deletedPointsIndex;
  std::vector<glm::dvec3> basisPoints;
  for (int idx: *basisPointsIndexPtr) {
    deletedPointsIndex.insert(idx);
    basisPoints.push_back((*verticesPtr)[idx]);
  }

  // Search for the neighboring points of basis points
  // within the range of averageDistance * 2.0
  auto octree = getPointCloud()->getOctree();
  double averageDistance = getPointCloud()->getAverageDistance();
  for (glm::dvec3 p: basisPoints) {
    // Search for the neighboring points of p
    std::vector<int>    hitPointIndices;
    std::vector<float>  hitPointDistances;
    int hitPointCount = octree->radiusSearch(
      pcl::PointXYZ(p.x, p.y, p.z),
      averageDistance * 2.0,
      hitPointIndices,
      hitPointDistances
    );

    // Add the hit points if the points are inside the sketch
    for (int idx: hitPointIndices) {
      glm::dvec3 hit_p = (*verticesPtr)[idx];
      glm::dvec3 hit_n = (*normalsPtr)[idx];

      // Cast a ray from p to cameraOrig onto screen.
      Ray ray(hit_p, getCameraOrig());
      Ray::Hit hitInfo = ray.castPointToPlane(getScreen());
      assert(hitInfo.hit == true);

      // If the point is outside the sketch or the normal is not directed to cameraOrig, then skip it.
      glm::dvec3 mapped_p = getScreen()->mapCoordinates(hitInfo.pos);
      if (insideSketch(mapped_p.x, mapped_p.y) && glm::dot(hit_n, hit_p - getCameraOrig()) < 0.0) deletedPointsIndex.insert(idx);
    }
  }

  // 1. Delete basis points from point cloud
  // 2. Update point cloud
  //    - update environments
  //    - update octree
  //    - render points and normals
  getPointCloud()->deletePoints(deletedPointsIndex);
  getPointCloud()->updatePointCloud(true);

  // Disable:
  //  - surface points
  //  - sketch
  disablePointCloud(SurfacePointName);
  disableSketch(SketchPrefix);

  // Reset all member variables
  resetSketch();
}