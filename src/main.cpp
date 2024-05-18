#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <math.h>

#include "camera.hpp"
#include "shader.hpp"
#include "point_cloud.hpp"
#include "sketch.hpp"
#include "holefill_tool.hpp"
#include "eraser_tool.hpp"

// window
const float InitialWindowWidth = 1600;
int windowWidth = 1600;
int windowHeight = 900;

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 2.0f), &windowWidth, &windowHeight);

int main() {
  /* Initialize GLFW */
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_SAMPLES, 4);

  GLFWwindow* window = glfwCreateWindow(windowWidth, windowHeight, "PointCloudEditor", NULL, NULL);
  if (window == NULL) {
    std::cout << "ERROR::GLFW::WINDOW_CREATION_FAILED" << std::endl;
    glfwTerminate();
    return -1;
  }
  
  // Make the context of our window the main context 
  // on the current thread.
  glfwMakeContextCurrent(window); 

  // Pass GLAD the function to load the address of the OpenGL function pointers
  // which is OS-specific.
  // glfwGetProcAddress defines the correct function based on which OS we're compiling for.
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "ERROR::GLAD::INITIALIZATION_FAILED" << std::endl;
    return -1;
  }

  /* Construct Point Cloud Class */
  Shader pointCloudShader(
    "../shader/shader_point_cloud.vert",
    "../shader/shader_point_cloud.geom", 
    "../shader/shader_point_cloud.frag"
  );
  PointCloud pointCloud(&pointCloudShader, "../data/bunny.obj", true);

  /* Construct Sketch Class */
  Shader sketchShader(
    "../shader/shader_sketch.vert",
    "../shader/shader_sketch.frag"
  );
  Sketch sketch(&sketchShader);

  /* Construct Tool Classes */
  int currentMode = MODE_NONE;
  HoleFillTool holeFillTool(&currentMode, &pointCloud, &sketch, &camera);
  EraserTool eraserTool(&currentMode, &pointCloud, &sketch, &camera);

  /* Setup ImGui */
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window, true); // Second param install_callback=true will install GLFW callbacks and chain to existing ones.
  ImGui_ImplOpenGL3_Init();

  bool pointCloudEnabled = true;

  /* Rendering loop */
  while(!glfwWindowShouldClose(window)) {
    /* Start the ImGui frame */
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    ImGuiIO& io = ImGui::GetIO();

    /* Setup rendering */
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glfwGetWindowSize(window, &windowWidth, &windowHeight);
    glViewport(0, 0, windowWidth, windowHeight);

    /* Camera Control */
    // if mouse is hovering a window or the state is under tool operation, then skip
    if (!ImGui::IsWindowHovered(1 << 2) && currentMode == MODE_NONE) {
      // camera.processMousePan(); // TODO: Enable pan function
      camera.processMouseMove();
      camera.processMouseScroll();
    }

    /* Show the demo window */
    // ImGui::ShowDemoWindow();

    /* Show the Menu window */
    ImGui::Begin("Menu Window"); 

    // Frame Rate
    ImGui::SetNextItemOpen(true);
    if (ImGui::CollapsingHeader("Frame Rate")) {
      ImGui::Text("Frame Rate:  %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
    }

    // Camera Params
    ImGui::SetNextItemOpen(true);
    if (ImGui::CollapsingHeader("Camera Parameters")) {
      float theta = camera.getTheta();
      float phi = camera.getPhi();
      float zoom = camera.getZoom();
      glm::vec3 trans = camera.getTrans();

      glm::vec3 worldPosition = camera.getWorldPosition();
      glm::vec3 worldDirection = camera.getWorldDirection();

      ImGui::Text("Theta (rad): %.2f", theta);
      ImGui::Text("Phi (rad):   %.2f", phi);
      ImGui::Text("Zoom:        %.2f", zoom);
      ImGui::Text("Translate:        (%.2f, %.2f, %.2f)", trans.x, trans.y, trans.z);
      ImGui::Text("Camera Position:  (%.2f, %.2f, %.2f)", worldPosition.x, worldPosition.y, worldPosition.z);
      ImGui::Text("Camera Direction: (%.2f, %.2f, %.2f)", worldDirection.x, worldDirection.y, worldDirection.z);
    }

    ImGui::Text("Test");

    // Screen Params
    ImGui::SetNextItemOpen(true);
    if (ImGui::CollapsingHeader("Screen Parameters")) {
      int windowWidth = camera.getWindowWidth();
      int windowHeight = camera.getWindowHeight();
      float screenWidth = camera.getScreenWidth();
      float screenHeight = camera.getScreenHeight();

      glm::vec3 screenOrigin = camera.getScreenOrigin();
      glm::vec3 screen_u = camera.getScreen_u();
      glm::vec3 screen_v = camera.getScreen_v();

      ImGui::Text("Window Size: (%d, %d)", windowWidth, windowHeight);
      ImGui::Text("Screen Size: (%.2f, %.2f)", screenWidth, screenHeight);
      ImGui::Text("Origin:  (%.2f, %.2f, %.2f)", screenOrigin.x, screenOrigin.y, screenOrigin.z);
      ImGui::Text("u-coord: (%.2f, %.2f, %.2f)", screen_u.x, screen_u.y, screen_u.z);
      ImGui::Text("v-coord: (%.2f, %.2f, %.2f)", screen_v.x, screen_v.y, screen_v.z);
    }

    // Sketch
    ImGui::SetNextItemOpen(true);
    if (ImGui::CollapsingHeader("Sketch")) {
      ImGuiIO& io = ImGui::GetIO();
      float xpos = 2.0f*io.MousePos.x/windowWidth - 1.0f;
      float ypos = 1.0f - 2.0f*io.MousePos.y/windowHeight;

      ImGui::Text("Screen Pos:       (%.2f, %.2f)", xpos, ypos);
    }

    // Render objects
    ImGui::SetNextItemOpen(true);
    if (ImGui::CollapsingHeader("Rendering")) {
      /* MVP Matrix */
      glm::mat4 model = camera.getModelMatrix();
      glm::mat4 view = camera.getViewMatrix();
      glm::mat4 projection = camera.getPerspectiveMatrix();

      /* Render Point Cloud */
      ImGui::Checkbox("Point Cloud", pointCloud.getEnabled());
      if (*pointCloud.getEnabled()) {
        glm::vec3 frontColor = glm::vec3(0.0f, 0.296f, 0.8f);
        glm::vec3 backColor = glm::vec3(0.6f, 0.5f, 0.0f);
        pointCloud.render(model, view, projection, frontColor, backColor);
      }

      /* Render Hole-Fill Tool Selection */
      ImGui::Checkbox("Hole-Fill Tool Selection", holeFillTool.getBasisPointCloud()->getEnabled());
      if (*holeFillTool.getBasisPointCloud()->getEnabled()) {
        glm::vec3 frontColor = glm::vec3(1.0f);
        glm::vec3 backColor = glm::vec3(1.0f);
        holeFillTool.getBasisPointCloud()->render(model, view, projection, frontColor, backColor, true);
      }

      /* Render Eraser Tool Selection */
      ImGui::Checkbox("Eraser Tool Selection", eraserTool.getBasisPointCloud()->getEnabled());
      if (*eraserTool.getBasisPointCloud()->getEnabled()) {
        glm::vec3 frontColor = glm::vec3(1.0f);
        glm::vec3 backColor = glm::vec3(1.0f);
        eraserTool.getBasisPointCloud()->render(model, view, projection, frontColor, backColor, true);
      }
    }

    // Tool selection
    ImGui::SetNextItemOpen(true);
    if (ImGui::CollapsingHeader("Tools")) {
      // Undo/Redo
      ImVec2 UndoRedoButtonSize = ImVec2(95.0f, 50.0f);
      if (ImGui::Button("<< Undo", UndoRedoButtonSize)) pointCloud.executeUndo();
      ImGui::SameLine();
      if (ImGui::Button("Redo >>", UndoRedoButtonSize)) pointCloud.executeRedo();
      ImGui::NewLine();

      // buttons for mode selection
      ImVec2 ToolButtonSize = ImVec2(200.0f, 50.0f);
      if (ImGui::Button("Reset", ToolButtonSize)) {
        currentMode = MODE_NONE;
      }
      if (ImGui::Button("Hole-Fill", ToolButtonSize)) {
        currentMode = MODE_HOLE_FILL;
        holeFillTool.initTool();
      }
      if (ImGui::Button("Eraser", ToolButtonSize)) {
        currentMode = MODE_ERASER;
        eraserTool.initTool();
      }

      // mode switch
      switch (currentMode) {
        case MODE_NONE:
          ImGui::Text("Selected Tool: None");
          break;
        case MODE_HOLE_FILL:
          ImGui::Text("Selected Tool: Hole-Fill");
          holeFillTool.runTool();
          break;
        case MODE_ERASER:
          ImGui::Text("Selected Tool: Eraser");
          eraserTool.runTool();
          break;
      }
    }

    ImGui::End();

    /* Render ImGui windows */
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    /* Swap buffers and poll IO events */
    glfwSwapBuffers(window);
    glfwPollEvents();  
  }

  /* Shutdown GLFW and ImGui */
  glfwTerminate();
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  return 0;
}
