#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include <iostream>
#include <vector>

const unsigned int WINDOW_WIDTH = 1280;
const unsigned int WINDOW_HEIGHT = 720;
const float GRID_SIZE= 0.02f;

// Modle View Projection values
glm::mat4 M;    // Model matrix
float Fovy = 45.0f;                                  // Degree of zoom (deg)
float Znear = 3.0f;                                  // Near clipping plane
float Zfar = 100.0f;                                 // Far clipping plane

glm::mat4 V;    // View matrix
glm::vec3 Camera = glm::vec3(0.0f, 0.0f, 3.0f);    // Camera position
glm::vec3 View = glm::vec3(0.0f, 0.0f, 0.0f);       // Viewing position
glm::vec3 Upward = glm::vec3(0.0f, 1.0f, 0.0f);     // Upward direction of Camera

glm::mat4 P;    // Projetion matrix
glm::mat4 mvp;  // Model View Projection matrix

// Callback
void key_callback(GLFWwindow*, int, int, int, int);
void error_callback(int, const char*);
void cursor_position_callback(GLFWwindow*, double, double);
void mouse_button_callback(GLFWwindow*, int, int, int);

struct Vertex {
    float x, y;     // TODO: 2D -> 3D
    float r, g, b;  // TODO: send color information to shader with glUniform4f https://stackoverflow.com/questions/54583368/how-to-draw-multiple-objects-in-opengl-using-multiple-vao-and-vbo
};
const int VertexMaxSiz = 10000;
const int VertexSiz = 2500;
const int VertexEdge = 50;
int cur_vertexsiz = 2500;
std::vector<Vertex> vertices;
std::vector<Vertex> vertices_2;

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
    vertices.resize(VertexMaxSiz);
    for (int i = 0; i < VertexSiz; i++) {
        float x = i%VertexEdge, y = i/VertexEdge;
        vertices[i] = Vertex {
            x: GRID_SIZE * x - GRID_SIZE * VertexEdge * 0.5f,
            y: GRID_SIZE * y - GRID_SIZE * VertexEdge * 0.5f,
            r: 1.0f,
            g: 1.0f,
            b: 1.0f
        };
    }

    vertices_2.resize(VertexMaxSiz);
    for (int i = 0; i < VertexSiz; i++) {
        float x = i%VertexEdge, y = i/VertexEdge;
        vertices_2[i] = Vertex {
            x: GRID_SIZE * x,
            y: GRID_SIZE * y,
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
    GLuint vertex_buffer, vertex_buffer_2, vertex_shader, fragment_shader, program;
    GLint mvp_location, vpos_location, vcol_location;

    // Register vertices to GL_ARRAY_BUFFER
    glGenBuffers(1, &vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);
        // GL_STATIC_DRAW: The data store contents will be modified once and used many times as the source for GL drawing commands.
        // GL_DYNAMIC_DRAW: The data store contents will be modified repeatedly and used many times as the source for GL drawing commands. 
    glGenBuffers(1, &vertex_buffer_2);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_2);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * vertices_2.size(), vertices_2.data(), GL_DYNAMIC_DRAW);

    
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

    // Display window
    while (!glfwWindowShouldClose(window)) {
        float aspect;
        int width, height;
 
        glfwGetFramebufferSize(window, &width, &height);
        aspect = width / (float) height;
 
        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT);

        P = glm::perspective(Fovy, aspect, Znear, Zfar);    // Projection
        V = glm::lookAt(Camera, View, Upward);              // View
        M = glm::mat4(1.0f);                                // Model
        // M = glm::rotate(M, (float)glfwGetTime(), glm::vec3(0.f, 0.f, 1.f)); // Rotate Model with z-axis
        mvp = P * V * M;

        // Execute program
        glUseProgram(program);
        glUniformMatrix4fv(mvp_location, 1, GL_FALSE, glm::value_ptr(mvp));

        // Bind vertex_buffer
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
        glVertexAttribPointer(vpos_location, 2, GL_FLOAT, GL_FALSE, sizeof(vertices[0]), (void*) 0);
        glVertexAttribPointer(vcol_location, 3, GL_FLOAT, GL_FALSE, sizeof(vertices[0]), (void*) (sizeof(float) * 2));
        glDrawArrays(GL_POINTS, 0, cur_vertexsiz);

        // Draw vertex_buffer_2
        // glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_2);
        // glVertexAttribPointer(vpos_location, 2, GL_FLOAT, GL_FALSE, sizeof(vertices_2[0]), (void*) 0);
        // glVertexAttribPointer(vcol_location, 3, GL_FLOAT, GL_FALSE, sizeof(vertices_2[0]), (void*) (sizeof(float) * 2));
        // glDrawArrays(GL_POINTS, 0, cur_vertexsiz);

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
        case GLFW_KEY_Z:
            if (action == GLFW_PRESS) {
                cur_vertexsiz = VertexSiz;
            }
            break;
    }
}

void error_callback(int error, const char* description) {
    fprintf(stderr, "Error: %s\n", description);
}

bool Dragging = false;
void cursor_position_callback(GLFWwindow *window, double xpos, double ypos) {
    if (Dragging) {
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        glm::vec3 wolrd_pos = glm::unProject(
            glm::vec3((float)xpos, (float)height - (float)ypos, 0.0f),  // 0.0(Znear) - 1.0(Zfar)
            M,
            P,
            glm::vec4(0.0f, 0.0f, (float)width, (float)height)
        );
        vertices[cur_vertexsiz] = Vertex {
            x: wolrd_pos.x,
            y: wolrd_pos.y,
            r: 1.0f,
            g: 1.0f,
            b: 1.0f
        };
        cur_vertexsiz = std::min(VertexMaxSiz, cur_vertexsiz + 1);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vertex)*cur_vertexsiz, vertices.data());
    }
}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods) {
        // button:  GLFW_MOUSE_BUTTON_1-8
        // action:  GLFW_PRESS, GLFW_RELEASE
        // mods:    GLFW_MOD_SHIFT, GLFW_MOD_CONTROL, GLFW_MOD_ALT ...

    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        std::cout << "Dragging" << std::endl;
        Dragging = true;
    } else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
        std::cout << "Released" << std::endl;
        Dragging = false;
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
        cur_vertexsiz = std::max(0, cur_vertexsiz - 1);
    }
}