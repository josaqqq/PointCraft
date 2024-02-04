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

    // Render greedy surface and pseudo surface.
    // Pseudo surface around holes in greedy surface is colored red.
    // Return the number of points on hole boundary.
    //  - greedyName: The name for greedy surface
    //  - pseudoname: The name for pseudo surface
    //  - averageDistance:  
    //      For greedy surface, used to determine the search radius
    //      For pseudo surface, the radius of the shown hexagons
    //  - greedyEnabled: If true, enable the registered greedy surface
    //  - pseudoEnabled: If true, enable the registered pseudo surface
    int renderPointCloudSurface(
      std::string greedyName,
      std::string pseudoName,
      double averageDistance, 
      bool greedyEnabled,
      bool pseudoEnabled
    );

    // Compute Greedy Projection and then render the reconstructed mesh.
    // Return the vertex indices on the hole boundaries.
    //  - name: The name of the registered surface
    //  - averageDistance:  used to determin the search radius
    //  - enabled:  If true, enable the registered greedy surface
    std::set<int> renderGreedySurface(
      std::string name,
      double averageDistance, 
      bool enabled
    );

    // Show hexagons for each vertex as a pseudo surface.
    //  - name: The name of the registered surface
    //  - averageDistance:  the radius of the shown hexagon.
    //  - enabled:  If true, enable the registered pseudo surface
    //  - boundaryVerticesIdx: The indices of the vertices on the hole boundary.
    void renderPseudoSurface(
      std::string name,
      double averageDistance, 
      bool enabled,
      const std::set<int> &boundaryVerticesIdx = std::set<int>{}
    );

  private:
    std::vector<glm::dvec3> *Vertices;
    std::vector<glm::dvec3> *Normals; 

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