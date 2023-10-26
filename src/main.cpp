#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include <iostream>
#include <vector>
#include <set>

#include "model.hpp"
#include "geometry.hpp"

float INF = 100000.0f;
float EPS = 1e-5;

// Window
const unsigned int WINDOW_WIDTH = 1280;
const unsigned int WINDOW_HEIGHT = 720;
const float GRID_SIZE= 0.04f;
bool Sketching = false;

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

// Vertices
const int VBO_MAX_SIZ = 1000000;
const int VERTEX_SIZ = 2500;
const int VERTEX_EDGE = 50;

GLuint vertex_buffer; 
int vertex_siz = 0;
std::vector<Vertex> vertices(VBO_MAX_SIZ);
std::set<std::pair<int, int>> vertices_set;
std::vector<int> vertices_history;

GLuint sketch_buffer;
int sketch_siz = 0;
std::vector<Vertex> sketch(VBO_MAX_SIZ);

// Shader
static const char* vertex_shader_text =
"#version 140\n"
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
"#version 140\n"
"varying vec3 color;\n"
"void main()\n"
"{\n"
"    gl_FragColor = vec4(color, 1.0);\n"
"}\n";

int main() {
    // Vertex initialization
    for (int i = 0; i < VERTEX_SIZ; i++) {
        int grid_x = i%VERTEX_EDGE, grid_y = i/VERTEX_EDGE;

        float x = (float)GRID_SIZE * (float)grid_x - (float)GRID_SIZE * (float)VERTEX_EDGE * 0.5f;
        float y = (float)GRID_SIZE * (float)grid_y - (float)GRID_SIZE * (float)VERTEX_EDGE * 0.5f;
        
        float f = x*x + y*y - 0.1f;
        float g = (x - 0.5f)*(x - 0.5f) + (y - 0.5f)*(y - 0.5f) - 0.15f;
        float h = (x + 0.5f)*(x + 0.5f) + (y - 0.5f)*(y - 0.5f) - 0.15f;
        float j = (x - 0.5f)*(x - 0.5f) + (y + 0.5f)*(y + 0.5f) - 0.15f;
        float k = (x + 0.5f)*(x + 0.5f) + (y + 0.5f)*(y + 0.5f) - 0.15f;

        if (f < 0.0f || g < 0.0f || h < 0.0f || j < 0.0f || k < 0.0f) {
            continue;
        }

        vertices[vertex_siz] = Vertex {
            x: x,
            y: y,
            r: 1.0f,
            g: 1.0f,
            b: 1.0f
        };
        vertices_set.insert({glm::round(x/GRID_SIZE), glm::round(y/GRID_SIZE)});
        vertex_siz++;
    }
    vertices_history.push_back(vertex_siz);

    // Check the computer environment
    if (!glfwInit()) {
        std::cerr << "Failed to initialize glfw." << std::endl;
        return -1;
    }

    // Set the context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    // TODO: Enable this part
    // glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);


    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "PointCraft", NULL, NULL);
        // Window and monitor are not specified
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

    // Load the extensions of OpenGL
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        glfwTerminate();
        return -1;
    }

    const GLubyte* renderer = glGetString(GL_RENDERER);
    const GLubyte* version = glGetString(GL_VERSION);

    std::cout << "Renderer:\t\t\t" << renderer << std::endl;
    std::cout << "OpenGL version supported:\t" << version << std::endl;

    // Buffer & Shader & Program
    GLuint vertex_shader, fragment_shader, program;
    GLint mvp_location, vpos_location, vcol_location;

    // Register vertices to GL_ARRAY_BUFFER
    glGenBuffers(1, &vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * VBO_MAX_SIZ, vertices.data(), GL_DYNAMIC_DRAW);
        // GL_STATIC_DRAW: The data store contents will be modified once and used many times as the source for GL drawing commands.
        // GL_DYNAMIC_DRAW: The data store contents will be modified repeatedly and used many times as the source for GL drawing commands. 

    // Register sketch to GL_ARRAY_BUFFER
    glGenBuffers(1, &sketch_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, sketch_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * VBO_MAX_SIZ, NULL, GL_DYNAMIC_DRAW);

    // Register vertex_shader_text to GL_VERTEX_SHADER
    vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_shader_text, NULL);
    glCompileShader(vertex_shader);
    GLint vertex_shader_compiled;
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &vertex_shader_compiled);
    if (vertex_shader_compiled == GL_FALSE) {
        std::cerr << "Failed to compile vertex shader" << std::endl;
        glfwTerminate();
        return -1;
    }
    
    // Register fragment_shader_text to GL_FRAGMENT_SHADER
    fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_shader_text, NULL);
    glCompileShader(fragment_shader);
    GLint fragment_shader_compiled;
    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &fragment_shader_compiled);
    if (fragment_shader_compiled == GL_FALSE) {
        std::cerr << "Failed to compile fragment shader" << std::endl;
        glfwTerminate();
        return -1;
    }
 
    // Register program
    program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);
    GLint is_linked;
    glGetProgramiv(program, GL_LINK_STATUS, &is_linked);
    if (is_linked == GL_FALSE) {
        std::cerr << "Failed to link shaders" << std::endl;
        glfwTerminate();
        return -1;
    }

    // Set the variables in shader_text
    mvp_location = glGetUniformLocation(program, "MVP");    // ModelViewProjection
    vpos_location = glGetAttribLocation(program, "vPos");   // (x, y)
    vcol_location = glGetAttribLocation(program, "vCol");   // (r, g, b)
    glEnableVertexAttribArray(vpos_location);
    glEnableVertexAttribArray(vcol_location);

    // Display window
    glPointSize(3.0f);
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

        // Draw vertices
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
        glVertexAttribPointer(vpos_location, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*) 0);
        glVertexAttribPointer(vcol_location, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*) (sizeof(float) * 2));
        glDrawArrays(GL_POINTS, 0, vertex_siz);

        // Draw sketch
        if (Sketching) {
            glBindBuffer(GL_ARRAY_BUFFER, sketch_buffer);
            glVertexAttribPointer(vpos_location, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*) 0);
            glVertexAttribPointer(vcol_location, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*) (sizeof(float) * 2));
            // glDrawArrays(GL_LINE_LOOP, 0, sketch_siz);  // Draw a not closed line
            glDrawArrays(GL_LINE_STRIP, 0, sketch_siz); // Draw a closed line
        }

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
                if (vertices_history.size() != 1) {
                    int prev_count = *vertices_history.rbegin();
                    vertices_history.pop_back();
                    int cur_count = *vertices_history.rbegin();
                    for (int i = cur_count; i < prev_count; i++) {
                        vertices_set.erase({glm::round(vertices[i].x/GRID_SIZE), glm::round(vertices[i].y/GRID_SIZE)});
                    }
                }
                vertex_siz = *vertices_history.rbegin();
            }
            break;
    }
}

void error_callback(int error, const char* description) {
    fprintf(stderr, "Error: %s\n", description);
}

void cursor_position_callback(GLFWwindow *window, double xpos, double ypos) {
    if (Sketching) {
        // Window coordinates into world coordinates
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        glm::vec3 wolrd_pos = glm::unProject(
            glm::vec3((float)xpos, (float)height - (float)ypos, 0.0f),  // 0.0(Znear) - 1.0(Zfar)
            M,
            P,
            glm::vec4(0.0f, 0.0f, (float)width, (float)height)
        );

        // Skip if distance from previous point is not sufficient
        if (sketch.size() != 0) {
            glm::vec2 prev_vertex = glm::vec2(sketch[sketch_siz - 1].x, sketch[sketch_siz - 1].y);
            glm::vec2 cur_vertex = glm::vec2(wolrd_pos.x, wolrd_pos.y);
            if (glm::distance(prev_vertex, cur_vertex) < GRID_SIZE) return;
        }

        // Add vertex to sketch
        sketch[sketch_siz] = Vertex {
            x: wolrd_pos.x,
            y: wolrd_pos.y,
            r: 1.0f,
            g: 0.0f,
            b: 0.0f
        };
        sketch_siz = std::min(VBO_MAX_SIZ, sketch_siz + 1);
        glBindBuffer(GL_ARRAY_BUFFER, sketch_buffer);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vertex)*sketch_siz, sketch.data());
    }
}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods) {
        // button:  GLFW_MOUSE_BUTTON_1-8
        // action:  GLFW_PRESS, GLFW_RELEASE
        // mods:    GLFW_MOD_SHIFT, GLFW_MOD_CONTROL, GLFW_MOD_ALT ...

    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        std::cout << "Sketching" << std::endl;

        Sketching = true;
    } else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
        std::cout << "Released" << std::endl;

        // Calculate the bound box of sketch
        float min_x = INF, max_x = -INF;
        float min_y = INF, max_y = -INF;
        for (int i = 0; i < sketch_siz; i++) {
            min_x = std::min(min_x, sketch[i].x);
            max_x = std::max(max_x, sketch[i].x);
            min_y = std::min(min_y, sketch[i].y);
            max_y = std::max(max_y, sketch[i].y);
        }

        // Add points to vertices
        for (int i = glm::floor(min_x/GRID_SIZE); i < glm::ceil(max_x/GRID_SIZE); i++) {
            for (int j = glm::floor(min_y/GRID_SIZE); j < glm::ceil(max_y/GRID_SIZE); j++) {
                // Skip if (i, j) is already contained
                if (vertices_set.count({i, j})) continue;

                float x = GRID_SIZE * i;
                float y = GRID_SIZE * j;

                // Internal/External judgements
                if (!inside_polygon(x, y, sketch, sketch_siz)) continue;

                // Add vertex to vertices
                vertices[vertex_siz] = Vertex {
                    x: x,
                    y: y,
                    r: 1.0f,
                    g: 1.0f,
                    b: 1.0f
                };
                vertices_set.insert({i, j});
                vertex_siz = std::min(VBO_MAX_SIZ, vertex_siz + 1);

                glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
                glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vertex)*vertex_siz, vertices.data());
            }
        }
        if (*vertices_history.rbegin() != vertex_siz) vertices_history.push_back(vertex_siz);

        Sketching = false;
        sketch_siz = 0;
    }
}