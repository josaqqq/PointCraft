#include "polyscope/polyscope.h"

#include "rbf.hpp"

RBF::RBF(
  PointCloud              *pointCloud,
  Plane                   *screen,
  std::vector<int>        *basisPointsIndex,
  std::vector<glm::dvec3> *discretizedPoints
) : pointCloud(pointCloud), screen(*screen), discretizedPoints(discretizedPoints) {
  sampleSize = basisPointsIndex->size();
  castedSize = discretizedPoints->size();

  // initialize basisPoints
  basisPoints.resize(sampleSize);
  for (int i = 0; i < sampleSize; i++) {
    int idx = (*basisPointsIndex)[i];

    basisPoints[i] = glm::dvec3(
      pointCloud->meshV(idx, 0),
      pointCloud->meshV(idx, 1),
      pointCloud->meshV(idx, 2)
    );
  }

  // Calculate average depth from screen to basisPoints
  double averageDepth = 0.0;
  for (int i = 0; i < sampleSize; i++) {
    averageDepth += screen->mapCoordinates(basisPoints[i]).z;
  }
  averageDepth /= sampleSize;

  // Calculate average plane for basis Points
  averagePlane = Plane(screen->getOrigin() + averageDepth*screen->getNormal(), screen->getNormal());

  // Cast discretizedPoints on screen to averagePlane
  const glm::dvec3 cameraOrig = polyscope::view::getCameraWorldPosition();
  for (int i = 0; i < castedSize; i++) {
    // Cast a ray for each discretized point
    Ray ray(cameraOrig, (*discretizedPoints)[i]);
    Hit hitInfo = ray.castPointToPlane(&averagePlane);
    if (!hitInfo.hit) continue;
    (*discretizedPoints)[i] = averagePlane.mapCoordinates(hitInfo.pos);
  }
}

void RBF::calcInterpolateSurface() {
  H = Eigen::MatrixXd::Zero(sampleSize, sampleSize);
  for (int i = 0; i < sampleSize; i++) {
    for (int j = 0; j < sampleSize; j++) {
      if (i == j) continue;

      glm::dvec3 castedBasis_i = averagePlane.mapCoordinates(basisPoints[i]);
      glm::dvec3 castedBasis_j = averagePlane.mapCoordinates(basisPoints[j]);
      H(i, j) = greenFunction(castedBasis_i, castedBasis_j);
    }
  }

  y = Eigen::VectorXd(sampleSize);
  for (int i = 0; i < sampleSize; i++) {
    y(i) = averagePlane.mapCoordinates(basisPoints[i]).z;
  }

  w = H.colPivHouseholderQr().solve(y);
}

Eigen::MatrixXd RBF::castPointsToSurface() {
  Eigen::MatrixXd castedPoints(castedSize, 3); 

  // for each discretized point
  for (int i = 0; i < castedSize; i++) {
    // for each green function
    double height = 0.0;
    for (int j = 0; j < sampleSize; j++) {
      glm::dvec3 castedBasis = averagePlane.mapCoordinates(basisPoints[j]);
      height += w(j)*greenFunction((*discretizedPoints)[i], castedBasis);
    }

    glm::dvec3 castedPoint = averagePlane.unmapCoordinates(
      glm::dvec3((*discretizedPoints)[i].x, (*discretizedPoints)[i].y, height)
    );
    castedPoints.row(i) <<
      castedPoint.x,
      castedPoint.y,
      castedPoint.z;
  }

  return castedPoints;
}

double RBF::greenFunction(glm::dvec3 x_i, glm::dvec3 x_j) {
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