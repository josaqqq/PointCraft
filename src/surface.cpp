#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>

#include "surface.hpp"

std::vector<Vertex> Surface::reconstructPoissonSurface() {
	/* Initialize point cloud */
	pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud(new pcl::PointCloud<pcl::PointNormal>);
	inputCloud->points.resize(vertices->size());
	for (size_t i = 0; i < vertices->size(); i++) {
		inputCloud->points[i].x = (*vertices)[i].x;
		inputCloud->points[i].y = (*vertices)[i].y;
		inputCloud->points[i].z = (*vertices)[i].z;

		inputCloud->points[i].normal_x = (*vertices)[i].nx;
		inputCloud->points[i].normal_y = (*vertices)[i].ny;
		inputCloud->points[i].normal_z = (*vertices)[i].nz;
	}

	/* Initialize poisson surface reconstruction */
	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setDepth(PoissonMaxDepth);
	poisson.setInputCloud(inputCloud);

	/* Reconstruct surface */
	pcl::PolygonMesh mesh;
	poisson.performReconstruction(mesh);

	/* Extract vertex information */
	pcl::PointCloud<pcl::PointXYZ>::Ptr meshVertices(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *meshVertices);
	std::vector<Vertex> newVertices(meshVertices->size());
  for (size_t i = 0; i < meshVertices->size(); i++) {
		newVertices[i].x = meshVertices->points[i].x;
		newVertices[i].y = meshVertices->points[i].y;
		newVertices[i].z = meshVertices->points[i].z;
  }

  /* Extract face information */
  std::vector<pcl::Vertices> meshFaces = mesh.polygons;
  std::vector<std::vector<size_t>> meshF(meshFaces.size(), std::vector<size_t>(3));
  for (size_t i = 0; i < meshFaces.size(); i++) {
    pcl::Vertices face = meshFaces[i];
    assert(face.vertices.size() == 3);
    for (size_t j = 0; j < face.vertices.size(); j++) {
      meshF[i][j] = face.vertices[j];
    }
  }

  /* Calculate the points' average normals */
  std::map<int, glm::vec3> indexToNormalSum;
  std::map<int, int>        indexToAdjacentCount;

	// Compute average noramls for each vertex
  for (size_t i = 0; i < meshF.size(); i++) {
    // Calculate the normal of the triangle
		Vertex vertex_u = newVertices[meshF[i][0]];
		Vertex vertex_v = newVertices[meshF[i][1]];
		Vertex vertex_w = newVertices[meshF[i][2]];

		glm::vec3 u = glm::vec3(vertex_u.x, vertex_u.y, vertex_u.z);
		glm::vec3 v = glm::vec3(vertex_v.x, vertex_v.y, vertex_v.z);
		glm::vec3 w = glm::vec3(vertex_w.x, vertex_w.y, vertex_w.z);

    glm::vec3 n = glm::normalize(glm::cross(v - u, w - u));
    for (int j = 0; j < 3; j++) {
      int vertexIdx = meshF[i][j];
      indexToNormalSum[vertexIdx] += n;
      indexToAdjacentCount[vertexIdx]++;
    }
  }

	// Set computed normals to return value
	for (size_t i = 0; i < newVertices.size(); i++) {
		glm::vec3 averaged_n = indexToNormalSum[i]/(float)indexToAdjacentCount[i];
		newVertices[i].nx = averaged_n.x;
		newVertices[i].ny = averaged_n.y;
		newVertices[i].nz = averaged_n.z;
	}

	return newVertices;
}