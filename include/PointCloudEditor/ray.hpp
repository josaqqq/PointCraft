#pragma once

#include "camera.hpp"
#include "point_cloud.hpp"

// Ray from camera origin to a point
class Ray {
	public:
		// Ray from camera origin to {xScreen, yScreen}
		Ray(Camera* camera, float xScreen, float yScreen);	
		// Ray from origin camera to p
		Ray(Camera* camera, glm::vec3 p);		

		struct Hit {
			Hit() {}

			bool hit = false;	// hit or not
			int index = -1;		// point index of pointCloud

			glm::vec3 rayDir = glm::vec3(0.0f, 0.0f, 0.0f);		// ray direction
			glm::vec3 pos = glm::vec3(0.0f, 0.0f, 0.0f);			// hit point
			glm::vec3 normal = glm::vec3(0.0f, 0.0f, 0.0f);		// hit point normal
		};

		// Process ray-marching in the direction from camera to the specified point
		// 	- pointCloud:	search target
		//	- searchRadiusOnScreen: Radius of cross section by secreen of search area
		Hit rayMarching(PointCloud* pointCloud, float searchRadiusOnScreen);

		// Compute the intersection between the ray and the screen
		Hit castPointToScreen();

	private:
		Camera* camera;	

		glm::vec3 rayDir = glm::vec3(0.0f, 0.0f, 0.0f);			// Vector of the direction of this ray
	
		// Camera params
		glm::vec3 cameraOrig = glm::vec3(0.0f, 0.0f, 0.0f);	// Coordinates of the origin of this ray
		glm::vec3 cameraDir = glm::vec3(0.0f, 0.0f, 0.0f);	// Vector of the direction of the camera

		const float maxDepth = 1e3f;
		const int maxStep = 300;
};