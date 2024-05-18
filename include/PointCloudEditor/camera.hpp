#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// Camera object
// The camera is fixed on z-axis and directed to (0.0, 0.0, 0.0)
// and the up direction is (0.0, 1.0, 0.0).
class Camera {
	public:
		// constructor
		Camera(glm::vec3 position, int* windowWidth, int* windowHeight);

		// returns the model, view, and perspective
		glm::mat4 getModelMatrix();
		glm::mat4 getViewMatrix();
		glm::mat4 getPerspectiveMatrix();

		// processs input received from a mouse event
		void updateCameraParams();
		void processMouseMove();
		void processMousePan();
		void processMouseScroll();

    // Map a point on the screen
		// between world coordinates <-> screen coordinates
    glm::vec2 mapWorldToScreen(glm::vec3 p);
    glm::vec3 mapScreenToWorld(glm::vec2 p);

		// returns the private values
		glm::vec3 getPosition();
		float getWindowWidth();
		float getWindowHeight();

		float getTheta();
		float getPhi();
		glm::vec3 getTrans();
		float getZoom();

		glm::vec3 getWorldPosition();
		glm::vec3 getWorldDirection();

		glm::vec3 getScreenOrigin();
		glm::vec3 getScreen_u();
		glm::vec3 getScreen_v();
		float getScreenWidth();
		float getScreenHeight();

		glm::vec3 getUp();
		float getNearClip();
		float getFarClip();
	
	private:
		glm::vec3 position;
		int* windowWidth;
		int* windowHeight;

		// rotate/pan/zoom event
		float theta = 0.0;
		float phi = 0.0;
		glm::vec3 trans = glm::vec3(0.0f, 0.0f, 0.0f);
		float zoom = 30.0f;

		// Coordinates in world coordinates
		glm::vec3 worldPosition = glm::vec3(0.0f, 0.0f, 0.0f);	// camera position in world coordinates
		glm::vec3 worldDirection = glm::vec3(0.0f, 0.0f, 0.0f);	// screen normal in world coordinates

		// Screen params
		glm::vec3 screenOrigin = glm::vec3(0.0f, 0.0f, 0.0f);		// screen origin in world coordinates
		glm::vec3 screen_u = glm::vec3(0.0f, 0.0f, 0.0f);				// screen basis for width direction in world coordinates
		glm::vec3 screen_v = glm::vec3(0.0f, 0.0f, 0.0f);				// screen basis for height direction in world coordinates
		float screenWidth = 0.0f;		// screen width in world coordinates
		float screenHeight = 0.0f;	// screen height in world coordinates

		// Constant member variables
		const glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
		const float nearClip = 0.1f;
		const float farClip = 100.0f;

		const float rotateSensitivity = 1.0f;
		const float panSensitivity = 2.0f;
};