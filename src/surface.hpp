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
    //  - enabled:  If true, enable the registered poisson surface
    std::pair<std::vector<glm::dvec3>, std::vector<std::vector<size_t>>> reconstructPoissonSurface(
      double averageDistance,
      bool enabled
    );

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

    // Compute Greedy Projection and then render the reconstructed mesh.
    //  - averageDistance:  used to determin the search radius
    //  - enabled:  If true, enable the registered greedy surface
    void showGreedyProjection(double averageDistance, bool enabled);

    // Show hexagons for each vertex as a pseudo surface.
    //  - averageDistance:  the radius of the shown hexagon.
    //  - enabled:  If true, enable the registered pseudo surface
    void showPseudoSurface(double averageDistance, bool enabled);

  private:
    std::string     Name;
    std::vector<glm::dvec3> *Vertices;
    std::vector<glm::dvec3> *Normals; 

    // Initialize MLS and then return pcl::MLSResult
    //  - searchRadius: the range of the nearest neighbor search
    pcl::MLSResult InitializeMLSSurface(double searchRadius);
};