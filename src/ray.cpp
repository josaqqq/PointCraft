#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "ray.hpp"

// Ray from camera origin to {xScreen, yScreen}
Ray::Ray(Camera* camera, float xScreen, float yScreen) : camera(camera) {
	cameraOrig = camera->getWorldPosition();
	cameraDir = camera->getWorldDirection();

	rayDir = glm::normalize(camera->mapScreenToWorld(glm::vec2(xScreen, yScreen)) - cameraOrig);
}

// Ray from camera origin to p
Ray::Ray(Camera* camera, glm::vec3 p) : camera(camera) {
	cameraOrig = camera->getWorldPosition();
	cameraDir = camera->getWorldDirection();

	rayDir = glm::normalize(p - cameraOrig);
}

// Process ray-marching in the direction from camera to the specified point
// 	- pointCloud:	search target
//	- searchRadiusOnScreen: Radius of cross section by secreen of search area
Ray::Hit Ray::rayMarching(PointCloud* pointCloud, float searchRadiusOnScreen) {
	Hit hit;	// Return value

	float nearClip = camera->getNearClip();
	float windowWidth = camera->getWindowWidth();
	float screenWidth = camera->getScreenWidth();
	float windowHeight = camera->getWindowHeight();
	float screenHeight = camera->getScreenHeight();

	std::vector<Vertex>* vertices = pointCloud->getVertices();
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* octree = pointCloud->getOctree();

	float currentDepth = 0.0f;
	float currentStep = 0;
	while (currentDepth < maxDepth && currentStep < maxStep) {
		/* Current center */
		glm::vec3 p = cameraOrig + currentDepth*rayDir;

		/* Search the nearest neighbor */
		std::vector<int>		hitPointIndices;
		std::vector<float>	hitPointDistances;
		int hitPointCount = octree->nearestKSearch(
			pcl::PointXYZ(p.x, p.y, p.z),
			1,
			hitPointIndices,
			hitPointDistances
		);
		if (hitPointCount == 0) {
			hit.hit = false;
			return hit;
		}

		/* Check if the nearest neighbor is inside the search range */
		// hit point values
		int hitIndex = hitPointIndices[0];
		Vertex hitVertex = (*vertices)[hitIndex];
		glm::vec3 hit_p = glm::vec3(hitVertex.x, hitVertex.y, hitVertex.z);
		glm::vec3 hit_n = glm::vec3(hitVertex.nx, hitVertex.ny, hitVertex.nz);

		float hitDistance = sqrt(hitPointDistances[0]);
		float hitDistanceToRay = glm::length(glm::cross(rayDir, hit_p - p));
		float hitDepth = glm::dot(rayDir, hit_p - cameraOrig);

		// search range
		float searchRadius = (searchRadiusOnScreen/nearClip)*hitDepth;

		// return, if the hitVertex satisfies conditions
		if (hitDistanceToRay < searchRadius) {
			if (glm::dot(hit_n, rayDir) < 0.0) {
				// the normal direct to the camera origin
				hit.hit = true;
				hit.index = hitIndex;

				hit.rayDir = rayDir;
				hit.pos = hit_p;
				hit.normal = hit_n;
			} else {
				// the normal does not direct to the camera origin
				hit.hit = false;
			}
			return hit;
		}

		/* Debug */
		// ImGui::Text("p: (%.4f, %.4f, %.4f)", p.x, p.y, p.z);
		// ImGui::Text("index: %d", hitIndex);
		// ImGui::Text("hit_p: (%.4f, %.4f, %.4f)", hit_p.x, hit_p.y, hit_p.z);
		// ImGui::Text("hit_n: (%.4f, %.4f, %.4f)", hit_n.x, hit_n.y, hit_n.z);
		// ImGui::Text("Hit Distance: 				%.4f", hitDistance);
		// ImGui::Text("Hit Distance to Ray: %.4f", hitDistanceToRay);
		// ImGui::Text("Hit Depth: 					%.4f", hitDepth);
		// ImGui::Text("Search Radius:				%.4f", searchRadius);
		// ImGui::NewLine();

		/* Proceed the ray */
		currentDepth += hitDistance;
		currentStep++;
	}	

	// No hit
	hit.hit = false;
	return hit;
}

Ray::Hit Ray::castPointToScreen() {
	Hit hit;	// Return value

  glm::vec3 screenOrigin = camera->getScreenOrigin();
  glm::vec3 screenNormal = cameraDir;

  // Calculate the intersection point
  float D = glm::dot(screenNormal, screenOrigin);
  float t = (D - glm::dot(screenNormal, cameraOrig))/glm::dot(screenNormal, rayDir);

	hit.hit = true;	// must hit
	hit.index = -1;

	hit.rayDir = rayDir;
	hit.pos = cameraOrig + t*rayDir;
	hit.normal = screenNormal;

	return hit;	
}