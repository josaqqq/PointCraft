// #pragma once

// #include "tool_util.hpp"

// class SprayTool: public ToolUtil {
// 	public:
// 		SprayTool(int* currentMode, PointCloud* pointCloud, Sketch* sketch, Camera* camera)
// 		: ToolUtil(currentMode, pointCloud, sketch, camera) {}
// 		~SprayTool() {}

// 		void runTool() override;

// 	private:
// 		void draggingEvent();
// 		void releasedEvent();

// 		// Constant member variables
// 		const int SprayKSize = 5;
// };