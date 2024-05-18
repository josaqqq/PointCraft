#pragma once

#include <vector>

#include "shader.hpp"

struct SketchPoint {
	float x, y;
};

class Sketch {
	public:
		Sketch(Shader* shader);
		virtual ~Sketch();

		// Send the buffer data and render the sketch
		void render(std::vector<SketchPoint>& vertices);

	private:
		// shader for sketch
		Shader* shader;
		unsigned int sketchVAO = 0;	// VAO object for sketch
		unsigned int sketchVBO = 0;	// VBO object for sketch

		// const member variables
		const float lineWidth = 3.0f;
};