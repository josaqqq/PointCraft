#pragma once

#include "tool_util.hpp"

class HoleFillTool : public ToolUtil {
	public:
		HoleFillTool(int* currentMode, PointCloud* pointCloud, Sketch* sketch, Camera* camera)
		: ToolUtil(currentMode, pointCloud, sketch, camera) {}
		~HoleFillTool() {}

		void runTool() override;

	private:
		void draggingEvent();
		void releasedEvent();

		/* Constant member variables */
		const int SurfacePointNum = 3;
};