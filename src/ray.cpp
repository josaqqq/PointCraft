#include <Eigen/Dense>

#include <iostream>

#include "ray.hpp"

Ray::Ray(double xScreen, double yScreen) {
  screenCoord = glm::vec2(xScreen, yScreen);
  orig        = polyscope::view::getCameraWorldPosition();
  rayDir      = polyscope::view::screenCoordsToWorldRay(glm::vec2(xScreen, yScreen));
  cameraDir   = polyscope::view::screenCoordsToWorldRay(glm::vec2(polyscope::view::windowWidth/2, polyscope::view::windowHeight/2));
}

Ray::Ray(double xScreen, double yScreen, PointCloud *pointCloud) : pointCloud(pointCloud) {
  screenCoord = glm::vec2(xScreen, yScreen);
  orig        = polyscope::view::getCameraWorldPosition();
  rayDir      = polyscope::view::screenCoordsToWorldRay(glm::vec2(xScreen, yScreen));
  cameraDir   = polyscope::view::screenCoordsToWorldRay(glm::vec2(polyscope::view::windowWidth/2, polyscope::view::windowHeight/2));
}

double Ray::calcDepthFromCamera(glm::vec3 point) {
  double d = -glm::dot(cameraDir, orig);
  double depth = std::abs(glm::dot(cameraDir, point) + d) / glm::length(cameraDir);
  return depth;
}

// Check whether this ray intersect
// with the sphere specified with 
// center and radius.
Hit Ray::checkSphere(glm::vec3 center, double radius) {
  Hit hitInfo(screenCoord);

  // Solve hit problem with quadratic equation.
  glm::vec3 oc = orig - center;
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
    hitInfo.pos = orig + static_cast<float>(hitInfo.t)*rayDir;
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
Hit Ray::searchNeighborPoints(double searchRadius) {
  searchRadius *= 2.0;
    // TODO: Somehow the scale is different... We need to check out the lengthScale.

  Hit hitInfo(screenCoord);

  double maxDepth = 100000.0;
  for (int i = 0; i < pointCloud->meshV.rows(); i++) {
    // point position
    glm::vec3 p = glm::vec3(
      pointCloud->meshV(i, 0),
      pointCloud->meshV(i, 1),
      pointCloud->meshV(i, 2)
    );
    // point normal
    glm::vec3 n = glm::vec3(
      pointCloud->meshN(i, 0),
      pointCloud->meshN(i, 1),
      pointCloud->meshN(i, 2)
    );

    // p is looked from the back side of the surface.
    if (glm::dot(rayDir, n) >= 0.0) continue;

    // p is outside of the scene.
    // TODO: Utilize nearClip, farClip
    double currDepth = glm::dot(p - orig, rayDir) / glm::length(rayDir);
    if (currDepth <= 0) continue;
    
    double currDist = glm::length(glm::cross(p - orig, rayDir)) / glm::length(rayDir); 
    if (currDist < searchRadius && currDepth < maxDepth) {
      maxDepth = currDepth;

      hitInfo.hit = true;
      hitInfo.t = currDepth;
      hitInfo.depth = calcDepthFromCamera(p);
      hitInfo.pos = p;
      hitInfo.normal = n;
    }
  }

  return hitInfo;
}

// Cast the point to the specified plane
// that is parallel to "camera plane".
// -> just approximation now.
Hit Ray::castPointToPlane(double depth) {
  Hit hitInfo(screenCoord);

  hitInfo.hit = true;
  hitInfo.t = depth;
  hitInfo.depth = depth;

  // TODO: we need to consider the true orig.
  hitInfo.pos = orig + (float)depth*rayDir;
  hitInfo.normal = -rayDir;

  return hitInfo;
}