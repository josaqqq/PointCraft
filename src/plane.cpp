#include <iostream>

#include "plane.hpp"

Plane::Plane(glm::dvec3 o, glm::dvec3 n) : o(o), n(n) {
  const double eps = 0.01d;
  n /= glm::length(n);

  // Initialize orthogonal basis by Gram-Schmidt process
  glm:;dvec3 e1, e2;
  e1 = glm::dvec3(1.0, 0.0, 0.0);
  if (std::abs(std::abs(glm::dot(e1, n)) - 1.0d) < eps)  {
    e1 = glm::dvec3(0.0, 1.0, 0.0);
  }
  e2 = glm::cross(n, e1);
  e2 /= glm::length(e2);

  // Gram-Schmidt process
  u = e1 - glm::dot(e1, n)*n;
  u /= glm::length(u);
  v = e2 - glm::dot(e2, n)*n - glm::dot(e2, u)*u;
  v /= glm::length(v);

  // Right-handed system
  if (glm::dot(glm::cross(u, v), n) < 0) {
    std::swap(u, v);
  }
}

// Map the point from {x, y, z} to {n, u, v}
glm::dvec3 Plane::mapCoordinates(glm::dvec3 p) {
  return glm::dvec3(
    glm::dot(p - o, u),
    glm::dot(p - o, v),
    glm::dot(p - o, n)
  );
}

// Map the point from {n, u, v} to {x, y, z}
glm::dvec3 Plane::unmapCoordinates(glm::dvec3 p) {
  return o + p.x*u + p.y*v + p.z*n;
}

// Get private member variables
glm::dvec3 Plane::getOrigin() {
  return o;
}
glm::dvec3 Plane::getNormal() {
  return n;
}