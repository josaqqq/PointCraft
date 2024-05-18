#version 330 core

in vec3 position;
in vec3 normal;

uniform mat4 model;
uniform mat4 view;

uniform vec3 frontColor;
uniform vec3 backColor;

// lighting position in Camera Space
const vec3 light_pos = vec3(12.0f, 10.0f, 20.0f);
const vec3 lightColor = vec3(1.0f);

out vec4 FragColor;

void main() {
  // Normal in Camera Space
  vec3 norm = normalize(mat3(transpose(inverse(view * model))) * normal);

  // Front and Back
  vec3 result;
  if (gl_FrontFacing) {
    result = frontColor;
  } else {
    result = backColor;
    norm *= -1.0f;
  }

  // Ambient
  float ambientStrength = 0.2f;
  vec3 ambient = ambientStrength * lightColor;

  // Diffuse
  vec3 lightDir = normalize(light_pos - position);
  float diff = max(dot(norm, lightDir), 0.0);
  vec3 diffuse = diff * lightColor;

  result *= (ambient + diffuse);
  FragColor = vec4(result, 0.8f);
}