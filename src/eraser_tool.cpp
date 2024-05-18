#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <iostream>
#include <time.h>

#include "eraser_tool.hpp"
#include "cluster.hpp"
#include "ray.hpp"

void EraserTool::runTool() {
	/* Render sketch points */
	sketch->render(sketchPoints);

	/* Process event */
	if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
		draggingEvent();
	} else {
		releasedEvent();
	}	
}

void EraserTool::draggingEvent() {
	ImGuiIO& io = ImGui::GetIO();

	/* Return, if there is no delta */
	bool hasDelta = io.MouseDelta.x != 0.0 || io.MouseDelta.y != 0.0;
	if (!hasDelta) return;

	/* Compute screen coordinate */
	float windowWidth = camera->getWindowWidth();
	float windowHeight = camera->getWindowHeight();
	float xpos = 2.0f*io.MousePos.x/windowWidth - 1.0f;
	float ypos = 1.0f - 2.0f*io.MousePos.y/windowHeight;

	/* Update sketch points */
	SketchPoint sketchPoint = SketchPoint{xpos, ypos};
	addSketchPoint(sketchPoint);
}

void EraserTool::releasedEvent() {
	if (sketchPoints.size() == 0) return;

	time_t timer = clock();
	std::cout << "<< Eraser Tool >>" << std::endl;

	/* Find basis points for the surface reconstruction */
	findBasisPoints(false, CLUSTER_MAX_SIZE);
	if (basisPoints.size() == 0) {
		resetTool();
		std::cout << "WARNING::ERASER_TOOL::NO_BASIS_POINT" << std::endl;
		return;
	}

	/* Search the point cloud for points to be deleted */
	std::set<int> deletedPointsIndex;

	auto octree = pointCloud->getOctree();
	auto vertices = pointCloud->getVertices();
	double averageDistance = pointCloud->getAverageDistance();
	for (Vertex v: basisPoints) {
		// Search for the neighboring pionts of p
		std::vector<int>		hitPointIndices;
		std::vector<float>	hitPointDistances;
		int hitPointCount = octree->radiusSearch(
			pcl::PointXYZ(v.x, v.y, v.z),
			averageDistance*2.0f,
			hitPointIndices,
			hitPointDistances
		);

		// Add the hit points if the points are inside the sketch
		for (int idx: hitPointIndices) {
			Vertex hit_v = (*vertices)[idx];
			glm::vec3 hit_p = glm::vec3(hit_v.x, hit_v.y, hit_v.z);
			glm::vec3 hit_n = glm::vec3(hit_v.nx, hit_v.ny, hit_v.nz);

			// Cast a ray from hit_v to the screen
			Ray ray(camera, hit_p);
			Ray::Hit hit = ray.castPointToScreen();
			glm::vec2 pos_screen = camera->mapWorldToScreen(hit.pos);
			if (
				insideSketch(SketchPoint{pos_screen.x, pos_screen.y}) &&
				glm::dot(hit_n, hit_p - camera->getWorldPosition()) < 0.0f
			)  {
				deletedPointsIndex.insert(idx);
			}
		}
	}

	/* Delete points from point cloud */
	pointCloud->deletePoints(deletedPointsIndex);

	/* Delte the selected points */
	resetTool();

	std::cout << "Elapsed Time: " << (double)(clock() - timer)/CLOCKS_PER_SEC << " s" << std::endl;
	std::cout << std::endl;
}