#pragma once

#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <set>
#include <stack>

#include <pcl/octree/octree_search.h>

#include "shader.hpp"

struct Vertex {
	float x, y, z;
	float nx, ny, nz;
};

class PointCloud {
	public:
		PointCloud();
		PointCloud(Shader* shader, std::string filepath, bool enabled = false);
		PointCloud(Shader* shader, std::vector<Vertex>& vertices, bool enabled = false);
		~PointCloud();

		// Update point cloud
		//	- update environments
		//	- update octree
		void updatePointCloud(bool clearPostEnv);

		// Render point cloud
		void render(
			glm::mat4 model, 
			glm::mat4 view, 
			glm::mat4 projection,
			glm::vec3 frontColor,
			glm::vec3 backColor,
			bool enableOffset = false
		);

		// Add vertices from the positions and normals
		void addPoints(std::vector<Vertex>& newVertices);

		// Delete vertices by referencing the vertex indices
		void deletePoints(std::set<int> &indices);

		// Execute Undo/Redo
		void executeUndo();
		void executeRedo();

		// Return member variables
		Shader* getShader();
		std::vector<Vertex>* getVertices();
		bool* getEnabled();
		glm::vec3 getCenter();
		double getScale();
		double getAverageDistance();
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* getOctree();

	private:
		// Constructor function
		void _PointCloud();

		// Move points to set the gravity point to (0.0, 0.0, 0.0),
		// and then scale the point cloud so that the bounding box side is 1.0
		void scalePointCloud();

		// Calculate average distance between the nearest points
		double calcAverageDistance();

		// Set updated vertices to octree
		void updateOctree();

		// shader for sketch
		Shader* shader;
		unsigned int pointCloudVAO = 0; // VAO object for point cloud
		unsigned int pointCloudVBO = 0; // VBO object for point cloud 

		// Point Cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud;
		std::vector<Vertex>	vertices;		// positions and normals
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree;	// octree for the point cloud

		bool updated = false;	// flag to check the point cloud to be sent to the buffer
		bool enabled = false;	// flag to check if the point cloud is rendered or not

		glm::vec3 center = glm::vec3(0.0f, 0.0f, 0.0f);	// Center of point cloud
		double scale						= 0.0;	// Scale of point cloud
    double averageDistance	= 0.0;	// Average Distance between a point and the nearest neighbor

    // Undo/Redo stacks
    std::stack<std::vector<Vertex>> prevEnvironments;
    std::stack<std::vector<Vertex>> postEnvironments;
};
