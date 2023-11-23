#include <Eigen/Dense>

#include <iostream>
#include <string>

#include "ray.hpp"
#include "plane.hpp"
#include "constants.hpp"

// Ray from { xScreen, yScreen }
Ray::Ray(double xScreen, double yScreen) {
  cameraOrig  = polyscope::view::getCameraWorldPosition();
  rayDir      = polyscope::view::screenCoordsToWorldRay(glm::vec2(xScreen, yScreen));
  cameraDir   = polyscope::view::screenCoordsToWorldRay(glm::vec2(polyscope::view::windowWidth/2, polyscope::view::windowHeight/2));
}

// Ray from p to q
Ray::Ray(glm::dvec3 p, glm::dvec3 q){
  cameraOrig  = polyscope::view::getCameraWorldPosition();
  rayDir      = glm::normalize(q - p);
  cameraDir   = polyscope::view::screenCoordsToWorldRay(glm::vec2(polyscope::view::windowWidth/2, polyscope::view::windowHeight/2));
}

// Check whether this ray intersect
// with the sphere specified with 
// center and radius.
Hit Ray::checkSphere(glm::dvec3 center, double radius) {
  // Regurn value
  Hit hitInfo;

  // Solve hit problem with quadratic equation.
  glm::dvec3 oc = cameraOrig - center;
  double a = glm::dot(rayDir, rayDir);
  double b = 2.0*glm::dot(oc, rayDir);
  double c = glm::dot(oc, oc) - radius*radius;
  double D = b*b - 4.0*a*c;

  if (D < 0.0) {
    // Ray does not hit the sphere.
    hitInfo.hit = false;
  } else {
    double t = (-b - glm::sqrt(D)) / (2.0*a);

    // Ray hits the sphere.
    hitInfo.hit = true;
    hitInfo.pos = cameraOrig + t*rayDir;
    hitInfo.normal = glm::normalize(hitInfo.pos - center);

    // Hit point is behind the scene.
    if (t < 0.0) hitInfo.hit = false;
  }
  
  return hitInfo;
}

// Search for the nearest neighbor point
// along the specified line through mesh.
Hit Ray::meshIntersection(Eigen::MatrixXd &meshV, Eigen::MatrixXi &meshF, PointCloud *pointCloud) {
  // Return value
  Hit hitInfo;

  // Check the intersection with bounding sphere
  // <- Point cloud is centered at (0, 0, 0)
  Hit hitSphere = checkSphere(glm::dvec3(0.0, 0.0, 0.0), pointCloud->getBoundingSphereRadius());
  if (!hitSphere.hit) return hitInfo;

  // Construct Octree
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
  inputCloud->points.resize(meshV.rows());
  for (int i = 0; i < meshV.rows(); i++) {
    inputCloud->points[i].x = meshV(i, 0);
    inputCloud->points[i].y = meshV(i, 1);
    inputCloud->points[i].z = meshV(i, 2);
  }
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(OctreeResolution);
  octree.setInputCloud(inputCloud);
  octree.addPointsFromInputCloud();

  // Search for the nearest neighbor traversing the ray.
  glm::dvec3 startPoint = hitSphere.pos + (double)1e-5*rayDir;
  int minDistIndex = -1;
  double minDist = 1e5;
  for (int i = 0; i < RayMaxStep; i++) {
    // current center of the searching area
    glm::dvec3 currentCenter = startPoint + (double)i*pointCloud->getAverageDistance()*rayDir;

    // Check whether currentCenter is in the searched area
    if (glm::length(currentCenter) > pointCloud->getBoundingSphereRadius()) break;

    // Search for nearest neighbors within the searchRadius.
    const int K = 1;
    std::vector<int>    hitPointIndices;
    std::vector<float>  hitPointDistances;
    int hitPointCount = octree.nearestKSearch(
      pcl::PointXYZ(currentCenter.x, currentCenter.y, currentCenter.z),
      K,
      hitPointIndices,
      hitPointDistances
    );

    // Update min value
    if (hitPointCount == 1 && sqrt(hitPointDistances[0]) < minDist) {
      minDistIndex = hitPointIndices[0];
      minDist = sqrt(hitPointDistances[0]);
    }
  }

  // If the ray hits no point, then return
  if (minDistIndex == -1) return hitInfo;

  // Search for faces adjacent to the selected vertex
  // and calculate the average normal.
  // -> Warning: Mesh must be triangle mesh
  int adjacentMeshCount = 0;
  glm::dvec3 averageNormal = glm::dvec3(0.0, 0.0, 0.0);
  for (int i = 0; i < meshF.rows(); i++) {
    bool adjacent = false;
    for (int j = 0; j < 3; j++) {
      if (meshF(i, j) == minDistIndex) adjacent = true;
    }
    if (!adjacent) continue;

    // this mesh is adjacent to the selected vertex.
    // then calculate normal
    adjacentMeshCount++;
    
    glm::dvec3 u = glm::dvec3(meshV(meshF(i, 0), 0), meshV(meshF(i, 0), 1), meshV(meshF(i, 0), 2));
    glm::dvec3 v = glm::dvec3(meshV(meshF(i, 1), 0), meshV(meshF(i, 1), 1), meshV(meshF(i, 1), 2));
    glm::dvec3 w = glm::dvec3(meshV(meshF(i, 2), 0), meshV(meshF(i, 2), 1), meshV(meshF(i, 2), 2));
    averageNormal += glm::normalize(glm::cross(v - u, w - u));
  }
  // If no point is adjacent to the selected point, then return
  if (adjacentMeshCount == 0) return hitInfo;
  averageNormal /= adjacentMeshCount;

  hitInfo.hit = true;
  hitInfo.index = minDistIndex;
  hitInfo.normal = averageNormal;

  // Cast the selected point onto the ray.
  glm::dvec3 minDistPoint = glm::dvec3(meshV(minDistIndex, 0), meshV(minDistIndex, 1), meshV(minDistIndex, 2));
  hitInfo.pos = cameraOrig + glm::dot(minDistPoint - cameraOrig, rayDir)*rayDir;

  return hitInfo;
}

// Search for the nearest neighbor point
// along the specified line through point cloud.
// The search range is within the range
// of the searchRadius.
std::vector<Hit> Ray::searchNeighborPoints(double searchRadius, PointCloud *pointCloud) {
  searchRadius *= polyscope::state::lengthScale;

  // Return value
  std::vector<Hit> hitsInfo;

  // Check the intersection with bounding sphere
  // <- Point cloud is centered at (0, 0, 0)
  Hit hitSphere = checkSphere(glm::dvec3(0.0, 0.0, 0.0), pointCloud->getBoundingSphereRadius());
  if (!hitSphere.hit) return hitsInfo;

  // Search for nearest neighbors traversing the ray
  glm::dvec3 startPoint = hitSphere.pos + (double)1e-5*rayDir;
  for (int i = 0; i < RayMaxStep; i++) {
    // current center of the searching area
    glm::dvec3 currentCenter = startPoint + (double)i*pointCloud->getAverageDistance()*rayDir;

    // Check whether currentCenter is in the searched area
    if (glm::length(currentCenter) > pointCloud->getBoundingSphereRadius()) break;

    // Search for nearest neighbors within the searchRadius.
    std::vector<int>    hitPointIndices;
    std::vector<float>  hitPointDistances;
    int hitPointCount = pointCloud->getOctree()->radiusSearch(
      pcl::PointXYZ(currentCenter.x, currentCenter.y, currentCenter.z),
      searchRadius,
      hitPointIndices,
      hitPointDistances
    );

    for (int j = 0; j < hitPointCount; j++) {
      Hit hitInfo;

      int hitIndex = hitPointIndices[j];
      hitInfo.hit = true;
      hitInfo.index = hitIndex;
      hitInfo.pos = glm::dvec3(
        pointCloud->Vertices(hitIndex, 0),
        pointCloud->Vertices(hitIndex, 1),
        pointCloud->Vertices(hitIndex, 2)
      );
      hitInfo.normal = glm::dvec3(
        pointCloud->Normals(hitIndex, 0),
        pointCloud->Normals(hitIndex, 1),
        pointCloud->Normals(hitIndex, 2)
      );

      // If the normal and camera direction are facing each other, 
      // then add the returned value
      if (glm::dot(hitInfo.normal, cameraDir) < 0.0) {
        hitsInfo.push_back(hitInfo);
      }
    }
  }

  return hitsInfo;
}

// Cast the point to the specified plane
// that is parallel to "camera plane".
Hit Ray::castPointToPlane(Plane* plane) {
  // Return value
  Hit hitInfo;

  double depth = std::abs(plane->mapCoordinates(cameraOrig).z);
  double t = depth/glm::dot(rayDir, cameraDir);

  hitInfo.hit = true;
  hitInfo.pos = cameraOrig + t*rayDir;
  hitInfo.normal = -cameraDir;

  return hitInfo;
}