#include "camera.hpp"

#include <iostream>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

// constructor
Camera::Camera(glm::vec3 position, int* windowWidth, int* windowHeight)
: position(position), windowWidth(windowWidth), windowHeight(windowHeight) {
	/* Update camera params */
	updateCameraParams();
}

// returns the model matrix and the view matrix
glm::mat4 Camera::getModelMatrix() {
	glm::mat4 model = glm::mat4(1.0f);
	model = glm::rotate(model, phi, glm::vec3(1.0f, 0.0f, 0.0f));
	model = glm::rotate(model, theta, glm::vec3(0.0f, 1.0f, 0.0f));
	return model;
}
glm::mat4 Camera::getViewMatrix() {
	return glm::lookAt(position + trans, trans, up);
}
glm::mat4 Camera::getPerspectiveMatrix() {
	return glm::perspective(
		glm::radians(zoom),
		(float)*windowWidth / (float)*windowHeight,
		nearClip, farClip
	);
}

// processs input received from a mouse event
void Camera::updateCameraParams() {
	/* worldPosition and worldNormal */
	glm::mat3 inverseMatrix = glm::inverse(getViewMatrix()*getModelMatrix());
	worldPosition = inverseMatrix*position;
	worldDirection = glm::normalize(trans - worldPosition);

	/* Screen params */
	screenOrigin = worldPosition + nearClip*worldDirection;
	screen_u = glm::normalize(glm::cross(worldDirection, up));
	screen_v = glm::normalize(glm::cross(screen_u, worldDirection));
	screenHeight = nearClip*std::tan(glm::radians(zoom/2.0f))*2.0f;
	screenWidth = screenHeight*static_cast<float>(*windowWidth)/static_cast<float>(*windowHeight);
}

void Camera::processMouseMove() {
	ImGuiIO& io = ImGui::GetIO();
	if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) return;
	if (ImGui::IsKeyDown(ImGuiKey_LeftCtrl) || ImGui::IsKeyDown(ImGuiKey_RightCtrl)) return;
	if (io.MouseDelta.x == 0.0 && io.MouseDelta.y == 0.0) return;

	/*
		- the left mouse button is down
		- Ctrl key is down
		- Mouse delta has a value
	*/

	/* Compute delta */
	float xdelta = io.MouseDelta.x;
	float ydelta = io.MouseDelta.y;
	glm::vec2 rotateDelta = glm::vec2(xdelta, ydelta)*rotateSensitivity;

	/* Rotate */
	const float pi = 3.141592653589;
	theta += 2.0f * pi * rotateDelta.x / *windowHeight; // height not width!
	phi 	+= 2.0f * pi * rotateDelta.y / *windowHeight;

	/* Restrict theta and phi */
	const float minAngle = -4.0f*pi;
	const float maxAngle = 4.0f*pi;
	if (theta < minAngle) 			theta += 2.0f*pi;
	else if (theta > maxAngle) 	theta -= 2.0f*pi;

	// TODO: Address the gimbal lock
	const float minPolarAngle = -pi/2.0f + 0.01f;
	const float maxPolarAngle = pi/2.0f - 0.01f;
	phi = std::max(minPolarAngle, std::min(maxPolarAngle, phi));

	/* Update camera params */
	updateCameraParams();
}

void Camera::processMousePan() {
	ImGuiIO& io = ImGui::GetIO();
	if (!ImGui::IsMouseDown(ImGuiMouseButton_Right)) return;
	if (ImGui::IsKeyDown(ImGuiKey_LeftCtrl) || ImGui::IsKeyDown(ImGuiKey_RightCtrl)) return;
	if (io.MouseDelta.x == 0.0 && io.MouseDelta.y == 0.0) return;

	/*
		- the left mouse button is down
		- Ctrl key is down
		- Mouse delta has a value
	*/

	/* Compute camera pan */
	float xdelta = io.MouseDelta.x / *windowWidth;
	float ydelta = io.MouseDelta.y / *windowHeight;
	trans += glm::vec3(-xdelta, ydelta, 0.0f)*panSensitivity;

	/* Update camera params */
	updateCameraParams();	
}

void Camera::processMouseScroll() {
	ImGuiIO& io = ImGui::GetIO();
	if (io.MouseWheel == 0.0) return;

	/*
		- Mouse wheel has a value
	*/

	/* Compute camera zoom */
	float yoffset = io.MouseWheel;
	zoom -= yoffset;
	if (zoom < 1.0f) 	zoom = 1.0f;
	if (zoom > 45.0f) zoom = 45.0f;

	/* Update camera params */
	updateCameraParams();	
}

// Map a point on the screen
// between world coordinates <-> screen coordinates
glm::vec2 Camera::mapWorldToScreen(glm::vec3 p) {
	glm::vec3 delta = p - worldPosition;
	float w = glm::dot(delta, screen_u)/(screenWidth/2.0f);
	float h = glm::dot(delta, screen_v)/(screenHeight/2.0f);
	return glm::vec2(w, h);
}
glm::vec3 Camera::mapScreenToWorld(glm::vec2 p) {
	return screenOrigin + p.x*(screenWidth/2.0f)*screen_u + p.y*(screenHeight/2.0f)*screen_v;
}

// returns the private values
glm::vec3 Camera::getPosition() {	return position; }
float Camera::getWindowWidth() { return static_cast<float>(*windowWidth); }
float Camera::getWindowHeight() {	return static_cast<float>(*windowHeight); }

float Camera::getTheta() { return theta; }
float Camera::getPhi() { return phi; }
glm::vec3 Camera::getTrans() { return trans; }
float Camera::getZoom() { return zoom; }

glm::vec3 Camera::getWorldPosition() { return worldPosition; }
glm::vec3 Camera::getWorldDirection() { return worldDirection; }

glm::vec3 Camera::getScreenOrigin() { return screenOrigin; }
glm::vec3 Camera::getScreen_u() { return screen_u; }
glm::vec3 Camera::getScreen_v() { return screen_v; }
float Camera::getScreenWidth() { return screenWidth; }
float Camera::getScreenHeight() { return screenHeight; }

glm::vec3 Camera::getUp() {	return up; }
float Camera::getNearClip() {	return nearClip;}
float Camera::getFarClip() {	return farClip; }
