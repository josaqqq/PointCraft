#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include <iostream>

const unsigned int WINDOW_WIDTH = 1280;
const unsigned int WINDOW_HEIGHT = 720;

void key_callback(GLFWwindow*, int, int, int, int);
void error_callback(int, const char*);
void cursor_position_callback(GLFWwindow*, double, double);
void mouse_button_callback(GLFWwindow*, int, int, int);

struct Vertex {
    float x, y;
    float r, g, b;
};
const int VertexSiz = 2500;
const int VertexEdge = 50;
Vertex vertices[VertexSiz];

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
    // Vertex initialization
    for (int i = 0; i < VertexSiz; i++) {
        float x = i/VertexEdge, y = i%VertexEdge;
        vertices[i] = Vertex {
            x: 0.02f * x - 0.5f,
            y: 0.02f * y - 0.5f,
            r: 1.0f,
            g: 1.0f,
            b: 1.0f
        };
    }

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
    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);

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
        glm::mat4 M, V, P, mvp;
 
        glfwGetFramebufferSize(window, &width, &height);
        ratio = width / (float) height;
 
        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT);

        // Projection
        P = glm::perspective(45.0f, ratio, 0.1f, 100.0f);
            // fovy(deg):   Degree of zoom (30 - 90)
            // aspect(w/h): Aspect ratio
            // znear:       Near clipping plane
            // zfar:        Far clipping plane
        // View
        V = glm::lookAt(
            glm::vec3(0.0, 0.0, -3.0),  // Camera position
            glm::vec3(0.0, 0.0, 0.0),   // Viewing position
            glm::vec3(0.0, 1.0, 0.0)    // Upward direction of Camera
        );  
        // Model
        M = glm::mat4(1.0f);
        // M = glm::rotate(M, (float)glfwGetTime(), glm::vec3(0.f, 0.f, 1.f)); // Rotate Model with z-axis
        mvp = P * V * M;

        // Execute program
        glUseProgram(program);
        glUniformMatrix4fv(mvp_location, 1, GL_FALSE, glm::value_ptr(mvp));
        glDrawArrays(GL_POINTS, 0, VertexSiz);

        glfwSwapBuffers(window);
        // glfwPollEvents()    // Processes only received events and then returns immediately
        glfwWaitEvents();   // Wait for receiving new input
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

void cursor_position_callback(GLFWwindow *window, double xpos, double ypos) {
    std::cout << xpos << " " << ypos << std::endl;
}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods) {
    // action: GLFW_PRESS or GLFW_RELEASE
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        std::cout << "PRESSED" << std::endl;
    }
}