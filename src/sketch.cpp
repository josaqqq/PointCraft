#include "sketch.hpp"

Sketch::Sketch(Shader* shader) : shader(shader) {
	/* Initialize VAO and VBO */
	glGenVertexArrays(1, &sketchVAO);	
	glGenBuffers(1, &sketchVBO);
}

Sketch::~Sketch() {
	glDeleteVertexArrays(1, &sketchVAO);
	glDeleteBuffers(1, &sketchVBO);

	std::cout << "Sketch was released!" << std::endl;
}

// Send the buffer data and render the sketch
void Sketch::render(std::vector<SketchPoint>& vertices) {
	// if there is no point, then return.
	if (vertices.size() == 0) return;

	/* Send data */
	shader->use();
	glBindVertexArray(sketchVAO);
	
	glBindBuffer(GL_ARRAY_BUFFER, sketchVBO);
	glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(SketchPoint), &vertices.front(), GL_DYNAMIC_DRAW);

	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);	

	/* Render sketch */
	glLineWidth(lineWidth);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glDrawArrays(GL_LINE_STRIP, 0, vertices.size());

	glDisable(GL_BLEND);
}

