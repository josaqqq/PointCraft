#pragma once

#include <string>
#include <Eigen/Dense>

class Surface {
  public:
    Surface(
      std::string Name,
      Eigen::MatrixXd *Vertices,
      Eigen::MatrixXd *Normals
    ) : Name(Name), Vertices(Vertices), Normals(Normals) {}

    // Reconstruct new surface with Vertices and Normals.
    //  - averageDistance: used to decide the resolution of Poisson Surface Reconstruction
    std::pair<Eigen::MatrixXd, Eigen::MatrixXi> reconstructPoissonSurface(double averageDistance);

    // Compute approximate surface using Vertices and Normals.
    // Then project points randomly onto the surface and return the projected points.
    //  - averageDistance:  the range of the randomly added points.
    //  - pointSize:        the size of randomly added points.
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> projectMLSSurface(double averageDistance, int pointSize);

    // Show hexagons for each vertex as a pseudo surface.
    //  - averageDistance:  the radius of the shown hexagon.
    void showPseudoSurface(double averageDistance);

  private:
    std::string     Name;
    Eigen::MatrixXd *Vertices;
    Eigen::MatrixXd *Normals; 
};
