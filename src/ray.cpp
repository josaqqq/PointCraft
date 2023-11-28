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

// Cast the point to the specified plane
// that is parallel to "camera plane".
Hit Ray::castPointToPlane(Plane* plane) {
  // Return value
  Hit hitInfo;

  double depth = std::abs(plane->mapCoordinates(cameraOrig).z);
  double t = depth/glm::dot(rayDir, cameraDir);

  hitInfo.hit = true; // must hit
  hitInfo.pos = cameraOrig + t*rayDir;
  hitInfo.normal = -cameraDir;

  return hitInfo;
}