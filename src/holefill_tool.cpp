#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <iostream>
#include <time.h>

#include "holefill_tool.hpp"
#include "ray.hpp"
#include "cluster.hpp"
#include "surface.hpp"

void HoleFillTool::runTool() {
	/* Render sketch points */
	sketch->render(sketchPoints);

	/* Process event */
	if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
		draggingEvent();
	} else {
		releasedEvent();
	}
}

void HoleFillTool::draggingEvent() {
	ImGuiIO& io = ImGui::GetIO();

	/* Return, if there is no delta */
	bool hasDelta = io.MouseDelta.x != 0.0 || io.MouseDelta.y != 0.0;
	if (!hasDelta) return;

	/* Compute screen coordinate */
	float windowWidth = camera->getWindowWidth();
	float windowHeight = camera->getWindowHeight();
	float xpos = 2.0f*io.MousePos.x/windowWidth - 1.0f;
	float ypos = 1.0f - 2.0f*io.MousePos.y/windowHeight;
	
	/* Update sketch and surface points*/
	SketchPoint sketchPoint = SketchPoint{xpos, ypos};
	addSketchPoint(sketchPoint);
	updateSurfacePoints(sketchPoint, SurfacePointNum);
}

void HoleFillTool::releasedEvent() {
	if (sketchPoints.size() == 0) return;

	time_t timer = clock();
	std::cout << "<< Hole-Fill Tool >>" << std::endl;

	/* Find basis points for the surface reconstruction */
	findBasisPoints(true, CLUSTER_MAX_SIZE);
	if (basisPoints.size() == 0) {
		resetTool();
		std::cout << "WARNING::HOLE_FILL_TOOL::NO_BASIS_POINT" << std::endl;
		return;
	}

	/* Compute Poisson Surface Reconstruction */
	Surface poissonSurface(&basisPoints);
	std::vector<Vertex> poissonVertices = poissonSurface.reconstructPoissonSurface();
	if (poissonVertices.size() == 0) {
		resetTool();
		std::cout << "WARNING::HOLE_FILL_TOOL::NO_POISSON_POINTS" << std::endl;
		return;
	}

	std::cout << "Basis Points Size: " << basisPoints.size() << std::endl;
	std::cout << "Poisson Points Size " << poissonVertices.size() << std::endl;

	/* Filter the reconstructed points in depth and voxel*/
	std::vector<Vertex> depthFilteredVertices = filterWithDepth(poissonVertices);
	std::vector<Vertex> voxelFilteredVertices = filterWithVoxel(depthFilteredVertices);

	/* Add the interpolated points */
	pointCloud->addPoints(voxelFilteredVertices);

	/* Reset all member variables */
	resetTool();

	std::cout << "Elapsed Time: " << (double)(clock() - timer)/CLOCKS_PER_SEC << " s" << std::endl;
	std::cout << std::endl;
}