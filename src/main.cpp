#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include <iostream>

const unsigned int WINDOW_WIDTH = 1280;
const unsigned int WINDOW_HEIGHT = 720;

void key_callback(GLFWwindow*, int, int, int, int);
void error_callback(int, const char*);

struct Vertex {
    float x, y;
    float r, g, b;
};
const int VertexSiz = 5;
const Vertex vertices[VertexSiz] = {
    {-0.6, -0.4, 1.0, 1.0, 1.0},
    {0.6, -0.4, 1.0, 1.0, 1.0},
    {0.0, 0.6, 1.0, 1.0, 1.0},
    {1.0, 0.8, 1.0, 1.0, 1.0},
    {1.0, -0.8, 1.0, 1.0, 1.0}
};

static const char* vertex_shader_text =
"#version 110\n"
"uniform mat4 MVP;\n"       // ModelViewProjection
"attribute vec2 vPos;\n"    // Vertex position
"attribute vec3 vCol;\n"    // Color information
"varying vec3 color;\n"     // Passed to fragment_shader
"void main()\n"             // Executed for each vertex
"{\n"
"    gl_Position = MVP * vec4(vPos, 0.0, 1.0);\n"
"    color = vCol;\n"
"}\n";
 
static const char* fragment_shader_text =
"#version 110\n"
"varying vec3 color;\n"
"void main()\n"
"{\n"
"    gl_FragColor = vec4(color, 1.0);\n"
"}\n";

int main() {
    // Check the computer environment
    if (!glfwInit()) {
        std::cerr << "Failed to initialize glfw." << std::endl;
        return -1;
    }

    // Set the context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "PointCraft", NULL, NULL);
        // window and monitor are not specified
    if (!window) {
        std::cerr << "Failed to create window." << std::endl;
        glfwTerminate();
        return -1;
    }
    
    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, key_callback);
    glfwSetErrorCallback(error_callback);

    // Load the functions of OpenGL
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        glfwTerminate();
        return -1;
    }

    // Buffer & Shader & Program
    GLuint vertex_buffer, vertex_shader, fragment_shader, program;
    GLint mvp_location, vpos_location, vcol_location;

    // Register vertices to GL_ARRAY_BUFFER
    glGenBuffers(1, &vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
        // GL_STATIC_DRAW: The data store contents will be modified once and used many times as the source for GL drawing commands.
        // GL_DYNAMIC_DRAW: The data store contents will be modified repeatedly and used many times as the source for GL drawing commands.
    
    // Register vertex_shader_text to GL_VERTEX_SHADER
    vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_shader_text, NULL);
    glCompileShader(vertex_shader);
    
    // Register fragment_shader_text to GL_FRAGMENT_SHADER
    fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_shader_text, NULL);
    glCompileShader(fragment_shader);
 
    // Register program
    program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);
    
    // Set the variables in shader_text
    mvp_location = glGetUniformLocation(program, "MVP");    // ModelViewProjection
    vpos_location = glGetAttribLocation(program, "vPos");   // (x, y)
    vcol_location = glGetAttribLocation(program, "vCol");   // (r, g, b)
    glEnableVertexAttribArray(vpos_location);
    glEnableVertexAttribArray(vcol_location);
    glVertexAttribPointer(vpos_location, 2, GL_FLOAT, GL_FALSE,
                          sizeof(vertices[0]), (void*) 0);
    glVertexAttribPointer(vcol_location, 3, GL_FLOAT, GL_FALSE,
                          sizeof(vertices[0]), (void*) (sizeof(float) * 2));

    // Display window
    while (!glfwWindowShouldClose(window)) {
        float ratio;
        int width, height;
        glm::mat4 m, v, p, mvp;
 
        glfwGetFramebufferSize(window, &width, &height);
        ratio = width / (float) height;
 
        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT);

        // Projection
        p = glm::perspective(45.0f, ratio, 0.1f, 100.0f);
            // fovy(deg):   Degree of zoom (30 - 90)
            // aspect(w/h): Aspect ratio
            // znear:       Near clipping plane
            // zfar:        Far clipping plane
        // View
        v = glm::lookAt(
            glm::vec3(0.0, 0.0, -3.0), // Camera position
            glm::vec3(0.0, 0.0, 0.0),   // Viewing position
            glm::vec3(0.0, 1.0, 0.0)    // Upward direction of Camera
        );  
        // Model
        m = glm::mat4(1.0f);
        m = glm::rotate(m, (float)glfwGetTime(), glm::vec3(0.f, 0.f, 1.f)); // Rotate Model with z-axis
        mvp = p * v * m;
    
        // Select the used program
        glUseProgram(program);
        glUniformMatrix4fv(mvp_location, 1, GL_FALSE, glm::value_ptr(mvp));
        glDrawArrays(GL_POINTS, 0, VertexSiz);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}

void key_callback(GLFWwindow *window, int key, int scannode, int action, int mods) {
    switch (key) {
        case GLFW_KEY_ESCAPE:
            if (action == GLFW_PRESS) {
                glfwSetWindowShouldClose(window, GLFW_TRUE);
            }
            break;
    }
}

void error_callback(int error, const char* description) {
    fprintf(stderr, "Error: %s\n", description);
}