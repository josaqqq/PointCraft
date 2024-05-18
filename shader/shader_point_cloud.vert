#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;

out vData {
  vec3 position;
  vec3 normal;
} vertex;

void main() {
  vertex.position = position;
  vertex.normal = normalize(normal);
}
