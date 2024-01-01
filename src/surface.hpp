#pragma once

#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>

#include <string>

class Surface {
  public:
    Surface(
      std::vector<glm::dvec3> *Vertices,
      std::vector<glm::dvec3> *Normals
    ) : Vertices(Vertices), Normals(Normals) {}

    // Reconstruct new surface with Vertices and Normals.
    //  - name: The name of the registered surface
    //  - averageDistance: used to decide the resolution of Poisson Surface Reconstruction
    //  - enabled:  If true, enable the registered poisson surface
    std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> reconstructPoissonSurface(
      std::string name,
      double averageDistance,
      bool enabled
    );

    // Compute an approximate surface using Vertices and Normals.
    // Then project points randomly onto the surface and return the projected points.
    //  - name: The name of the registered surface 
    //  - mousePos: screen coordinates of the mouse position
    //  - searchRadius: the range of the nearest neighbor search
    //  - averageDistance:  the radius of the range where points are randomly added.
    //  - pointSize:        the size of randomly added points.
    std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> projectMLSSurface(
      std::string name,
      glm::dvec2 mousePos,
      double searchRadius,
      double averageDistance, 
      int pointSize
    );

    // Show hexagons for each vertex as a pseudo surface.
    //  - name: The name of the registered surface
    //  - averageDistance:  the radius of the shown hexagon.
    //  - enabled:  If true, enable the registered pseudo surface
    void showPseudoSurface(
      std::string name,
      double averageDistance, 
      bool enabled
    );

  private:
    std::vector<glm::dvec3> *Vertices;
    std::vector<glm::dvec3> *Normals; 
};