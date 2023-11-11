#include <Eigen/Dense>

#include <iostream>

#include "ray.hpp"

extern Eigen::MatrixXd meshV;
extern Eigen::MatrixXd meshN;

Ray::Ray(double xScreen, double yScreen) {
  orig = polyscope::view::getCameraWorldPosition();
  dir = polyscope::view::screenCoordsToWorldRay(glm::vec2(xScreen, yScreen));
}

// Check whether this ray intersect
// with the sphere specified with 
// center and radius.
Hit Ray::checkSphere(glm::vec3 center, double radius) {
  Hit hitInfo;

  // Solve hit problem with quadratic equation.
  glm::vec3 oc = orig - center;
  double a = glm::dot(dir, dir);
  double b = 2.0*glm::dot(oc, dir);
  double c = glm::dot(oc, oc) - radius*radius;
  double D = b*b - 4.0*a*c;

  if (D < 0.0) {
    // Ray does not hit the sphere.
    hitInfo.hit = false;
  } else {
    hitInfo.hit = true;
    // Ray hits the sphere.
    hitInfo.t = (-b - glm::sqrt(D)) / (2.0*a);
    hitInfo.pos = orig + static_cast<float>(hitInfo.t)*dir;
    hitInfo.normal = glm::normalize(hitInfo.pos - center);

    // Hit point is behind the scene.
    if (hitInfo.t < 0.0) hitInfo.hit = false;
  }
  
  return hitInfo;
}

// Search for the nearest neighbor point
// along the specified line.
// The search range is within the range
// of the searchRadius.
Hit Ray::searchNeighborPoints(double searchRadius) {
  Hit hitInfo;

  double maxDist = searchRadius;
  for (int i = 0; i < meshV.rows(); i++) {
    // point position
    glm::vec3 p = glm::vec3(
      meshV(i, 0),
      meshV(i, 1),
      meshV(i, 2)
    );
    // point normal
    glm::vec3 n = glm::vec3(
      meshN(i, 0),
      meshN(i, 1),
      meshN(i, 2)
    );

    if (glm::dot(dir, n) >= 0.0)        continue; // p is looked from the back side of the surface.
    // TODO: This does not completely filter the points behind the scene.
    if (glm::dot(dir, p - orig) <= 0.0) continue; // p is behind the scene.
    
    // TODO: Use the previous selected points to determine the depth.
    double currDist = glm::length(glm::cross(p - orig, dir)); // Be aware that dir is needed to be normalized.
    if (currDist < maxDist) {
      maxDist = currDist;
      hitInfo.hit = true;
      hitInfo.pos = p;
      hitInfo.normal = n;
    }
  }

  return hitInfo;
}
