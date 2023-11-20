#pragma once

#include <glm/glm.hpp>
#include <Eigen/Dense>

#include "ray.hpp"
#include "plane.hpp"

class RBF {
  public:
    RBF(
      Plane *screen,
      std::vector<glm::dvec3> *basisPoints,
      std::vector<glm::dvec3> *discretizedPoints
    );

    // Calculate the interpolate surface with RBF network
    // using basis information
    void calcInterpolateSurface();

    // Cast the discretied points to the interpolate surface
    Eigen::MatrixXd castPointsToSurface();

  private:
    // Green function: |x_i - x_j|^2 * (log(|x_i - x_j|) - 1.0)
    // ref: https://www.fon.hum.uva.nl/praat/manual/biharmonic_spline_interpolation.html
    double greenFunction(glm::dvec3 x_i, glm::dvec3 x_j);

    void printDebug();

    int sampleSize;
    int castedSize;

    Plane screen;
    Plane averagePlane;

    std::vector<glm::dvec3> *basisPoints;
    std::vector<glm::dvec3> *discretizedPoints;

    Eigen::MatrixXd H;       // Matrix H(i, j) = g(x_i, x_j)
    Eigen::VectorXd w;       // Vector w(i) = w_i: weight vector
    Eigen::VectorXd y;       // Vector y(i) = y_i: height vector
};

