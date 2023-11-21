#include <Eigen/Dense>

#include <iostream>

#include "ray.hpp"
#include "plane.hpp"

// Ray from { xScreen, yScreen }
Ray::Ray(double xScreen, double yScreen) {
  orig        = polyscope::view::getCameraWorldPosition();
  rayDir      = polyscope::view::screenCoordsToWorldRay(glm::vec2(xScreen, yScreen));
  cameraDir   = polyscope::view::screenCoordsToWorldRay(glm::vec2(polyscope::view::windowWidth/2, polyscope::view::windowHeight/2));
}

// Ray from p to q
Ray::Ray(glm::dvec3 p, glm::dvec3 q){
  orig        = polyscope::view::getCameraWorldPosition();
  rayDir      = glm::normalize(q - p);
  cameraDir   = polyscope::view::screenCoordsToWorldRay(glm::vec2(polyscope::view::windowWidth/2, polyscope::view::windowHeight/2));
}

double Ray::calcDepthFromCamera(glm::dvec3 point) {
  double d = -glm::dot(cameraDir, orig);
  double depth = std::abs(glm::dot(cameraDir, point) + d) / glm::length(cameraDir);
  return depth;
}

// Check whether this ray intersect
// with the sphere specified with 
// center and radius.
Hit Ray::checkSphere(glm::dvec3 center, double radius) {
  Hit hitInfo;

  // Solve hit problem with quadratic equation.
  glm::dvec3 oc = orig - center;
  double a = glm::dot(rayDir, rayDir);
  double b = 2.0*glm::dot(oc, rayDir);
  double c = glm::dot(oc, oc) - radius*radius;
  double D = b*b - 4.0*a*c;

  if (D < 0.0) {
    // Ray does not hit the sphere.
    hitInfo.hit = false;
  } else {
    // Ray hits the sphere.
    hitInfo.hit = true;
    hitInfo.t = (-b - glm::sqrt(D)) / (2.0*a);
    hitInfo.pos = orig + hitInfo.t*rayDir;
    hitInfo.depth = calcDepthFromCamera(hitInfo.pos);
    hitInfo.normal = glm::normalize(hitInfo.pos - center);

    // Hit point is behind the scene.
    if (hitInfo.t < 0.0) hitInfo.hit = false;
  }
  
  return hitInfo;
}

// Search for the nearest neighbor point
// along the specified line.
// The search range is within the range
// of the searchRadius and the nearest
// point to the scene is chosen.
std::vector<Hit> Ray::searchNeighborPoints(double searchRadius, PointCloud *pointCloud) {
  std::vector<Hit> hitsInfo;

  searchRadius *= polyscope::state::lengthScale;
  double nearClip = polyscope::view::nearClipRatio*polyscope::state::lengthScale;
  double farClip = polyscope::view::farClipRatio*polyscope::state::lengthScale;

  for (int i = 0; i < pointCloud->meshV.rows(); i++) {
    // point position
    glm::dvec3 p = glm::dvec3(
      pointCloud->meshV(i, 0),
      pointCloud->meshV(i, 1),
      pointCloud->meshV(i, 2)
    );
    // point normal
    glm::dvec3 n = glm::dvec3(
      pointCloud->meshN(i, 0),
      pointCloud->meshN(i, 1),
      pointCloud->meshN(i, 2)
    );

    // p is looked from the back side of the surface.
    if (glm::dot(rayDir, n) >= 0.0) continue;

    // p is outside of the scene.
    double currDepth = glm::dot(p - orig, rayDir) / glm::length(rayDir);
    if (currDepth <= nearClip || farClip <= currDepth) continue;
    
    double currDist = glm::length(glm::cross(p - orig, rayDir)) / glm::length(rayDir); 
    if (currDist < searchRadius) {
      Hit hitInfo;

      hitInfo.hit = true;
      hitInfo.t = currDepth;
      hitInfo.depth = calcDepthFromCamera(p);

      hitInfo.index = i;
      hitInfo.pos = p;
      hitInfo.normal = n;

      hitsInfo.push_back(hitInfo);
    }
  }

  return hitsInfo;
}

// Cast the point to the specified plane
// that is parallel to "camera plane".
Hit Ray::castPointToPlane(Plane* plane) {
  Hit hitInfo;

  double depth = std::abs(plane->mapCoordinates(orig).z);

  hitInfo.hit = true;
  hitInfo.t = depth/glm::dot(rayDir, cameraDir);
  hitInfo.depth = depth;

  hitInfo.pos = orig + hitInfo.t*rayDir;
  hitInfo.normal = -cameraDir;

  return hitInfo;
}