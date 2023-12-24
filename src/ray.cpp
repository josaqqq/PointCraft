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

// Search pointCloud for the closest point to the screen
// within the range of searchRadius
Ray::Hit Ray::searchNearestNeighbor(PointCloud *pointCloud, double searchRadius) {
  // Return value
  Hit hitInfo;

  const double MaxDepth = 1e5;
  double currentDepth = MaxDepth;
  std::vector<glm::dvec3> *verticesPtr = pointCloud->getVertices();
  std::vector<glm::dvec3> *normalsPtr = pointCloud->getNormals();
  for (size_t i = 0; i < verticesPtr->size(); i++) {
    glm::dvec3 p = (*verticesPtr)[i];
    glm::dvec3 pn = (*normalsPtr)[i];

    // If the distance from the ray line is greater than searchRadius, then skip it.
    double distFromRayLine = glm::length(glm::cross(p - cameraOrig, rayDir));
    if (distFromRayLine >= searchRadius) continue;

    // If the depth is less than currenDepth, then update hitInfo
    double depthFromCameraOrig = glm::dot(p - cameraOrig, rayDir);
    if (depthFromCameraOrig < currentDepth) {
      currentDepth = depthFromCameraOrig;

      hitInfo.hit = true;
      hitInfo.rayDir = rayDir;
      
      hitInfo.index = i;
      hitInfo.pos = p;
      hitInfo.normal = pn;
    } 
  }

  if (currentDepth == MaxDepth) {
    hitInfo.hit = false;
    return hitInfo;
  }

  // If the normal of the hit point does not face rayDir, then return false
  if (hitInfo.hit && glm::dot(hitInfo.normal, rayDir) >= 0.0) {
    hitInfo.hit = false;
  }
  
  return hitInfo;
}

// Cast the point to the specified plane
Ray::Hit Ray::castPointToPlane(Plane* plane) {
  // Return value
  Hit hitInfo;

  glm::dvec3 planeOrig = plane->getOrigin();
  glm::dvec3 planeNormal = plane->getNormal();

  // Calculate the intersection point
  double D = glm::dot(planeNormal, planeOrig);
  double t = (D - glm::dot(planeNormal, cameraOrig))/glm::dot(planeNormal, rayDir);

  hitInfo.hit = true; // must hit
  hitInfo.rayDir = rayDir;

  hitInfo.index = -1;
  hitInfo.pos = cameraOrig + t*rayDir;
  hitInfo.normal = planeNormal;

  return hitInfo;
}