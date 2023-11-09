#include <iostream>

#include "ray.hpp"

Ray::Ray(double xScreen, double yScreen) {
  orig = polyscope::view::getCameraWorldPosition();
  dir = polyscope::view::screenCoordsToWorldRay(glm::vec2(xScreen, yScreen));

  // std::cout << "Ray Origin:\t" << orig.x << " " << orig.y << " " << orig.z << std::endl;
  // std::cout << "Ray Direction:\t" << dir.x << " " << dir.y << " " << dir.z << std::endl;
  // std::cout << std::endl;

  // polyscope::CameraParameters cameraParameters = polyscope::view::getCameraParametersForCurrentView();
  // glm::vec3 pos = cameraParameters.getPosition();
  // glm::vec3 lookDir = cameraParameters.getLookDir();
  // glm::vec3 upDir = cameraParameters.getUpDir();

  // std::cout << "Camera Position\t" << pos.x << " " << pos.y << " " << pos.z << std::endl;
  // std::cout << "Camera LookDir\t" << lookDir.x << " " << lookDir.y << " " << lookDir.z << std::endl;
  // std::cout << "Camera UpDir\t" << upDir.x << " " << upDir.y << " " << upDir.z << std::endl;
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