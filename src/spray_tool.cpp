// #include <imgui.h>
// #include <imgui_impl_glfw.h>
// #include <imgui_impl_opengl3.h>

// #include <iostream>
// #include <time.h>

// #include "spray_tool.hpp"
// #include "sketch.hpp"
// #include "cluster.hpp"
// #include "ray.hpp"

// void SprayTool::runTool() {
// 	/* Process event */
// 	if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
// 		draggingEvent();
// 	} else {
// 		releasedEvent();
// 	}
// }

// void SprayTool::draggingEvent() {
// 	ImGuiIO& io = ImGui::GetIO();

// 	/* Return, if there is no delta */
// 	bool hasDelta = io.MouseDelta.x != 0.0 || io.MouseDelta.y != 0.0;
// 	if (!hasDelta) return;

// 	/* Compute screen coordinate */
// 	float windowWidth = camera->getWindowWidth();
// 	float windowHeight = camera->getWindowHeight();
// 	float xpos = 2.0f*io.MousePos.x/windowWidth - 1.0f;
// 	float ypos = 1.0f - 2.0f*io.MousePos.y/windowHeight;

// 	/* Update sketch points */
// 	SketchPoint sketchPoint = SketchPoint{xpos, ypos};
// 	updateSurfacePoints(sketchPoint, SprayKSize);
// }

// void SprayTool::releasedEvent() {
// 	if (sketchPoint.size() == 0) return;

// 	time_t timer = clock();
// 	std::cout << "<< Spray Tool >>" << std::endl;

// 	/* Reset all member variables */
// 	resetTool();

// 	std::cout << "Elapsed Time: " << (double)(clock() - timer)/CLOCKS_PER_SEC << std::endl;
// 	std::cout << std::endl;
// }
