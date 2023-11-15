#pragma once

#include <glm/glm.hpp>
#include <Eigen/Dense>

#include "ray.hpp"

class RBF {
  public:
    RBF(
      double averageDepth,
      std::pair<glm::dvec3, glm::dvec3> orthoBasis,
      std::vector<Hit>        *boundaryPoints,
      std::vector<glm::dvec2> *boundaryOnPlane,
      std::vector<glm::dvec2> *discretizedPoints
    ) 
    : averageDepth(averageDepth),
      orthoU(orthoBasis.first), 
      orthoV(orthoBasis.second), 
      boundaryPoints(boundaryPoints),
      boundaryOnPlane(boundaryOnPlane), 
      discretizedPoints(discretizedPoints) {}

    // Calculate the interpolate surface with RBF network
    // using boundary information
    void calcInterpolateSurface();

    // Cast the discretied points to the interpolate surface
    Eigen::MatrixXd castPointsToSurface();

  private:
    // Green function: |x_i - x_j|^2 * (log(|x_i - x_j|) - 1.0)
    // ref: https://www.fon.hum.uva.nl/praat/manual/biharmonic_spline_interpolation.html
    double greenFunction(glm::dvec2 x_i, glm::dvec2 x_j);

    void printDebug();

    double averageDepth;
    glm::dvec3 orthoU, orthoV;
    
    std::vector<Hit>        *boundaryPoints;
    std::vector<glm::dvec2> *boundaryOnPlane;
    std::vector<glm::dvec2> *discretizedPoints;

    int sampleSize;
    int castedSize;

    Eigen::MatrixXd H;       // Matrix H(i, j) = g(x_i, x_j)
    Eigen::VectorXd w;       // Vector w(i) = w_i: weight vector
    Eigen::VectorXd y;       // Vector y(i) = y_i: height vector
};

