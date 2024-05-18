#pragma once

#include "tool_util.hpp"

class EraserTool: public ToolUtil {
	public:
		EraserTool(int* currentMode, PointCloud* pointCloud, Sketch* sketch, Camera* camera)
		: ToolUtil(currentMode, pointCloud, sketch, camera) {}
		~EraserTool() {}

		void runTool() override;

	private:
		void draggingEvent();
		void releasedEvent();
};