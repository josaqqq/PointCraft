#pragma once

#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>

#include <string>

class Surface {
  public:
    Surface(
      std::string Name,
      std::vector<glm::dvec3> *Vertices,
      std::vector<glm::dvec3> *Normals
    ) : Name(Name), Vertices(Vertices), Normals(Normals) {}

    // Reconstruct new surface with Vertices and Normals.
    //  - averageDistance: used to decide the resolution of Poisson Surface Reconstruction
    std::pair<std::vector<glm::dvec3>, std::vector<std::vector<size_t>>> reconstructPoissonSurface(double averageDistance);

    // Compute an approximate surface using Vertices and Normals.
    // Then project points randomly onto the surface and return the projected points.
    //  - xPos: io.DisplayFramebufferScale.x * mousePos.x
    //  - xPos: io.DisplayFramebufferScale.y * mousePos.y
    //  - searchRadius: the range of the nearest neighbor search
    //  - averageDistance:  the radius of the range where points are randomly added.
    //  - pointSize:        the size of randomly added points.
    std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> projectMLSSurface(
      int xPos,
      int yPos,
      double searchRadius,
      double averageDistance, 
      int pointSize
    );

    // Compute an approximate surface using Vertices and Normals.
    // Then project points inside of the range of basis points.
    //  - searchRadius: the range of the nearest neighbor search
    //  - averagedistance: the distance between points in the point cloud
    std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> reconstructMLSSurface(
      double searchRadius,
      double averageDistance
    );

    // Show hexagons for each vertex as a pseudo surface.
    //  - averageDistance:  the radius of the shown hexagon.
    void showPseudoSurface(double averageDistance);

  private:
    std::string     Name;
    std::vector<glm::dvec3> *Vertices;
    std::vector<glm::dvec3> *Normals; 

    // Initialize MLS and then return pcl::MLSResult
    //  - searchRadius: the range of the nearest neighbor search
    pcl::MLSResult InitializeMLSSurface(double searchRadius);
};