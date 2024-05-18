#pragma once

#include "point_cloud.hpp"

class Surface {
	public:
		Surface(std::vector<Vertex>* vertices) : vertices(vertices) {}

		// Reconstruct new surface with vertices and normals
		std::vector<Vertex> reconstructPoissonSurface();

	private:
		std::vector<Vertex>* vertices;

		/* Constant member variables */
		const int PoissonMaxDepth = 8;
		const int PoissonThreadNum = 4;
};