#version 330 core 

layout (points) in;
layout (triangle_strip, max_vertices = 24) out;

in vData {
	vec3 position;
	vec3 normal;
} vertices[];

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

uniform float radius;
uniform bool enableOffset;

out vec3 position;
out vec3 normal;

void main() {
	// Transformation Matrix
	mat4 mvp = projection * view * model;
	
	// Position and normal in Local Space
	position = vertices[0].position;
	normal = vertices[0].normal;
	
	// Initialize orthogonal basis by Gram-Schmidt process
	float eps = 0.005f;
	vec3 e1 = vec3(1.0f, 0.0f, 0.0f);
	if (abs(abs(dot(normal, e1)) - 1.0f) < eps) e1 = vec3(0.0f, 1.0f, 0.0f);
	vec3 e2 = normalize(cross(normal, e1));

	// Gram-Schmidt process
	vec3 u = normalize(e1 - dot(e1, normal)*normal);
	vec3 v = normalize(e2 - dot(e2, normal)*normal - dot(e2, u)*u);

	// Right-handed system
	if (dot(cross(u, v), normal) < 0.0f) {
		vec3 temp = u;
		u = v;
		v = temp;
	}

	// Render circle
	const float PI = 3.1415926535897932384626433832795;
	const int STEP_SIZE = 12;
	const float THETA_STEP = PI/STEP_SIZE;
	const float OFFSET_MAGNITUDE = 0.00001f;
	
	vec3 OFFSET = OFFSET_MAGNITUDE*normal;
	if (!enableOffset) {
		OFFSET *= 0.0f;
	}

	gl_Position = mvp * vec4(position + radius*u + OFFSET, 1.0f);
	EmitVertex();

	for (float theta = THETA_STEP; theta < PI; theta += THETA_STEP) {
		float x = radius * cos(theta);
		float y = radius * sin(theta);

		gl_Position = mvp * vec4(position + x*u + y*v + OFFSET, 1.0f);
		EmitVertex();

		gl_Position = mvp * vec4(position + x*u - y*v + OFFSET, 1.0f);
		EmitVertex();
	}

	gl_Position = mvp * vec4(position - radius*u + OFFSET, 1.0f);
	EmitVertex();

	EndPrimitive();
}