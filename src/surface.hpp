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

    // Flip faces if the average of the adjacent faces are inversed.
    void flipFaces(
      std::vector<glm::dvec3> &meshV,
      std::vector<std::vector<size_t>> &meshF
    ); 

    // Detect the holes on the mesh
    //  1.  Detect the edges not shared by two faces
    //  2.  Manage vertices of the detected edges with UnionFind
    //  3.  Calculate the total length of the hole, and skip 
    //      if the length of the hole boundary is less than 
    //      the threshold.
    //  4.  Return the boundary faces' indices
    //
    //  - meshV: Vertices on the mesh
    //  - meshF: Faces on the mesh
    //  - boundaryLengthLimit: Skip if the boundary length is less than this value
    std::set<int> detectHolesOnMesh(
      std::vector<glm::dvec3> &meshV,
      std::vector<std::vector<size_t>> &meshF,
      double boundaryLengthLimit
    );

    // UnionFind used in detectHolesOnMesh
    struct UnionFind {
      std::vector<int> par; 
      UnionFind(int N) : par(N) {
        for(int i = 0; i < N; i++) par[i] = i;
      }
      int root(int x) {
        if (par[x] == x) return x;
        return par[x] = root(par[x]);
      }
      void unite(int x, int y) {
        int rx = root(x); 
        int ry = root(y); 
        if (rx == ry) return; 
        par[rx] = ry; 
      }
      bool same(int x, int y) { 
        int rx = root(x);
        int ry = root(y);
        return rx == ry;
      }
    };
};