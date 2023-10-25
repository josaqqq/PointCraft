#pragma once

struct Vertex {
    float x, y;     // TODO: 2D -> 3D
    float r, g, b;  // TODO: send color information to shader with glUniform4f https://stackoverflow.com/questions/54583368/how-to-draw-multiple-objects-in-opengl-using-multiple-vao-and-vbo
};