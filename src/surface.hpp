#pragma once

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

    // Compute approximate surface using Vertices and Normals.
    // Then project points randomly onto the surface and return the projected points.
    //  - xPos: io.DisplayFramebufferScale.x * mousePos.x
    //  - xPos: io.DisplayFramebufferScale.y * mousePos.y
    //  - averageDistance:  the range of the randomly added points.
    //  - pointSize:        the size of randomly added points.
    std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> projectMLSSurface(
      int xPos,
      int yPos,
      double averageDistance, 
      int pointSize
    );

    // Show hexagons for each vertex as a pseudo surface.
    //  - averageDistance:  the radius of the shown hexagon.
    void showPseudoSurface(double averageDistance);

  private:
    std::string     Name;
    std::vector<glm::dvec3> *Vertices;
    std::vector<glm::dvec3> *Normals; 
};