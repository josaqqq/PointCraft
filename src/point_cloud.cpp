#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include "point_cloud.hpp"

PointCloud::PointCloud() : inputCloud(new pcl::PointCloud<pcl::PointXYZ>), octree(0.001) {}

PointCloud::PointCloud(Shader* shader, std::string filepath, bool enabled) 
: shader(shader), inputCloud(new pcl::PointCloud<pcl::PointXYZ>), octree(0.001), enabled(enabled) {
  /* Load point cloud from .obj file */
  std::ifstream objFile(filepath);
  if (!objFile.is_open()) {
    std::cout << "ERROR::POINTCLOUD::OBJ_READING_FAILED" << std::endl;
    return;
  }

  clock_t timer = clock();

  std::string line;
  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
  while (std::getline(objFile, line)) {
    std::istringstream iss(line);

    std::string token;
    iss >> token;

    if (token == "v") {
      float x, y, z;
      iss >> x >> y >> z;
      positions.emplace_back(glm::vec3(x, y, z));
    } else if (token == "vn") {
      float nx, ny, nz;
      iss >> nx >> ny >> nz;
      normals.emplace_back(glm::vec3(nx, ny, nz));
    }
  }
  objFile.close();

  assert(positions.size() == normals.size());
  const size_t verticesSize = positions.size();
  for (size_t i = 0; i < verticesSize; i++) {
    glm::vec3 p = positions[i];
    glm::vec3 pn = normals[i];
    vertices.emplace_back(Vertex{
      p.x, p.y, p.z, pn.x, pn.y, pn.z
    });
  }

  const double elapsed_time = static_cast<double>(clock() - timer) / CLOCKS_PER_SEC;
  std::cout << "Read obj file: " << elapsed_time << " ms" << std::endl;

  /* Setup point cloud class */
  _PointCloud();
}

PointCloud::PointCloud(Shader* shader, std::vector<Vertex>& vertices, bool enabled) 
: shader(shader), inputCloud(new pcl::PointCloud<pcl::PointXYZ>), vertices(vertices), octree(0.001), enabled(enabled) {
  /* Setup point cloud class */
  _PointCloud();
}

PointCloud::~PointCloud() {
  glDeleteVertexArrays(1, &pointCloudVAO);
  glDeleteBuffers(1, &pointCloudVBO);

  std::cout << "Point Cloud was released!" << std::endl;
}

// Update point cloud
//	- update environments
//  - update octree
void PointCloud::updatePointCloud(bool clearPostEnv) {
  /* Update environments */
  prevEnvironments.push(vertices);
  if (clearPostEnv) postEnvironments = std::stack<std::vector<Vertex>>();

  /* Update variables */
  updateOctree();
  updated = true;
}


// Render point cloud
void PointCloud::render(
  glm::mat4 model, 
  glm::mat4 view, 
  glm::mat4 projection,
  glm::vec3 frontColor,
  glm::vec3 backColor,
  bool enableOffset
) {
  // if there is no point, then return
  if (vertices.size() == 0) return;

  /* Send data to buffer */
  if (updated) {
    glBindVertexArray(pointCloudVAO);

    glBindBuffer(GL_ARRAY_BUFFER, pointCloudVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(Vertex), &vertices.front(), GL_STATIC_DRAW);
    
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);
  }

  /* Render point cloud */
  unsigned int shaderID = shader->use();
  glBindVertexArray(pointCloudVAO);

  shader->setMat4("model", model);
  shader->setMat4("view", view);
  shader->setMat4("projection", projection);
  shader->setVec3("frontColor", frontColor);
  shader->setVec3("backColor", backColor);
  shader->setFloat("radius", averageDistance);
  shader->setBool("enableOffset", enableOffset);

  glDrawArrays(GL_POINTS, 0, vertices.size());

  // Debug
  ImGui::Text("Shader Program: %d", shaderID);
  ImGui::Text("(VAO, VBO): (%d, %d)", pointCloudVAO, pointCloudVBO);
  ImGui::Text("Vertex Size: %d", vertices.size());
}

// Add vertices from the positions and normals
void PointCloud::addPoints(std::vector<Vertex>& newVertices) {
  /* Add new vertices to vertices */
  for (size_t i = 0; i < newVertices.size(); i++) {
    vertices.emplace_back(newVertices[i]);
  }

  /* Update point cloud */
  updatePointCloud(true);
}

// Delete vertices by referencing the vertex indices
void PointCloud::deletePoints(std::set<int> &indices) {
  if (indices.size() == 0) return;

  const int curSize = vertices.size();
  const int newSize = curSize - indices.size();

  /* Delete points from vertices */
  std::vector<Vertex> newVertices;
  std::set<int>::iterator itr = indices.begin();
  for (int i = 0; i < curSize; i++) {
    if (itr != indices.end() && i == *itr) {
      itr++;
    } else {
      newVertices.emplace_back(vertices[i]);
    }
  }
  vertices = newVertices;

  /* Update point cloud */
  updatePointCloud(true);
}

// Execute Undo/Redo
void PointCloud::executeUndo() {
  if (prevEnvironments.size() < 2) return;

  /* Undo event */
  // move the current env from prev-stack to post-stack
  std::vector<Vertex> currentEnv = prevEnvironments.top();
  prevEnvironments.pop();
  postEnvironments.push(currentEnv);

  // set new env to the prevstack
  std::vector<Vertex> newEnv = prevEnvironments.top();
  prevEnvironments.pop();
  vertices = newEnv;

  /* Update point cloud */
  updatePointCloud(false);
}
void PointCloud::executeRedo() {
  if (postEnvironments.size() < 1) return;

  /* Redo event */
  std::vector<Vertex> newEnv = postEnvironments.top();
  postEnvironments.pop();
  vertices = newEnv;

  /* Update point cloud */
  updatePointCloud(false);
}

// Return member variables
Shader* PointCloud::getShader() { return shader; }
std::vector<Vertex>* PointCloud::getVertices() { return &vertices; }
bool* PointCloud::getEnabled() { return &enabled; }
glm::vec3 PointCloud::getCenter() { return center; }
double PointCloud::getScale() { return scale; }
double PointCloud::getAverageDistance() { return averageDistance; }
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* PointCloud::getOctree() { return &octree; }

// Constructor function
void PointCloud::_PointCloud() {
  /* Initialize VAO and VBO */
  glGenVertexArrays(1, &pointCloudVAO);
  glGenBuffers(1, &pointCloudVBO);

  clock_t timer = clock();
  double elapsed_time = 0.0;

  /* Scale the input point cloud*/
  // scalePointCloud();

  /* Update point cloud */
  updatePointCloud(false);

  /* Calculate average distance between points */
  calcAverageDistance();
}

// Move points to set the gravity point to (0.0, 0.0, 0.0),
// and then scale the point cloud so that the bounding box side is 1.0
void PointCloud::scalePointCloud() {
  /* Move points to set the gravity points to (0.0, 0.0, 0.0) */
  for (size_t i = 0; i < vertices.size(); i++) {
    Vertex v = vertices[i];
    center += glm::vec3(v.x, v.y, v.z);
  }
  center /= static_cast<double>(vertices.size());

  /* Scale the point cloud so that the bounding box side is 1.0 */
  float boundingBoxSide = 0.0;
  for (size_t i = 0; i < vertices.size(); i++) {
    vertices[i].x -= center.x;
    vertices[i].y -= center.y;
    vertices[i].z -= center.z;

    boundingBoxSide = std::max(boundingBoxSide, std::abs(vertices[i].x));
    boundingBoxSide = std::max(boundingBoxSide, std::abs(vertices[i].y));
    boundingBoxSide = std::max(boundingBoxSide, std::abs(vertices[i].z));
  }

  float scale = 0.5f / boundingBoxSide;
  for (size_t i = 0; i < vertices.size(); i++) {
    vertices[i].x *= scale;
    vertices[i].y *= scale;
    vertices[i].z *= scale;
  }
}

// Calculate average distance between the nearest points
double PointCloud::calcAverageDistance() {
  const int K = 6;
  averageDistance = 0.0;
  int allHitPointCount = 0;

  for (size_t i = 0; i < vertices.size(); i++) {
    Vertex v = vertices[i];
    
    // Search the nearest neighbor 
    std::vector<int>    hitPointIndices;
    std::vector<float>  hitPointDistances;
    int hitPointCount = octree.nearestKSearch(
      pcl::PointXYZ(v.x, v.y, v.z),
      K,
      hitPointIndices,
      hitPointDistances
    );

    for (int i = 1; i < hitPointCount; i++) averageDistance += sqrt(hitPointDistances[i]);
    allHitPointCount += hitPointCount;
  }
  
  averageDistance /= allHitPointCount;
  return averageDistance;
}

// Update registered vertices
void PointCloud::updateOctree() {
  inputCloud->points.resize(vertices.size());
  for (size_t i = 0; i < vertices.size(); i++) {
    inputCloud->points[i].x = vertices[i].x;
    inputCloud->points[i].y = vertices[i].y;
    inputCloud->points[i].z = vertices[i].z;
  }

  octree.deleteTree();
  octree.setInputCloud(inputCloud);
  octree.addPointsFromInputCloud();
}
