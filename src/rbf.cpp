#include "polyscope/polyscope.h"

#include "rbf.hpp"

void RBF::calcInterpolateSurface() {
  sampleSize = boundaryOnPlane->size();

  H = Eigen::MatrixXd::Zero(sampleSize, sampleSize);
  for (int i = 0; i < sampleSize; i++) {
    for (int j = 0; j < sampleSize; j++) {
      if (i == j) continue;

      glm::dvec2 x_i = (*boundaryOnPlane)[i];
      glm::dvec2 x_j = (*boundaryOnPlane)[j];
      H(i, j) = greenFunction(x_i, x_j);
    }
  }

  y = Eigen::VectorXd(sampleSize);
  for (int i = 0; i < sampleSize; i++) {
    y(i) = (*boundaryPoints)[i].depth - averageDepth;
  }

  w = H.colPivHouseholderQr().solve(y);
}

Eigen::MatrixXd RBF::castPointsToSurface() {
  castedSize = discretizedPoints->size();
  const glm::dvec3 orig       = polyscope::view::getCameraWorldPosition();
  const glm::dvec3 cameraDir  = polyscope::view::screenCoordsToWorldRay(glm::vec2(polyscope::view::windowWidth/2, polyscope::view::windowHeight/2));

  Eigen::MatrixXd castedPoints(castedSize, 3); 

  // for each discretized point
  for (int i = 0; i < castedSize; i++) {
    // for each green function
    double height = 0.0;
    for (int j = 0; j < sampleSize; j++) {
      height += w(j)*greenFunction((*discretizedPoints)[i], (*boundaryOnPlane)[j]);
    }

    glm::dvec3 castedPoint = 
      orig + (*discretizedPoints)[i].x*orthoU + (*discretizedPoints)[i].y*orthoV + (height + averageDepth)*cameraDir;
    castedPoints.row(i) <<
      castedPoint.x,
      castedPoint.y,
      castedPoint.z;
  }

  return castedPoints;
}

double RBF::greenFunction(glm::dvec2 x_i, glm::dvec2 x_j) {
  double dist = glm::length(x_i - x_j);
  if (dist == 0) return 0.0;
  return dist * dist * (std::log(dist) - 1.0d);
}

void RBF::printDebug() {
  for (int i = 0; i < sampleSize; i++) {
    for (int j = 0; j < sampleSize; j++) {
      std::cout << H(i, j) << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;

  for (int i = 0; i < sampleSize; i++) {
    std::cout << y(i) << std::endl;
  }
  std::cout << std::endl;


  for (int i = 0; i < sampleSize; i++) {
    std::cout << w(i) << std::endl;
  }
  std::cout << std::endl;
}