#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

class Shader {
  public:
    // the program ID
    unsigned int ID;

    // Constructor reads and builds the shader
    Shader(
      const char* vertexPath,
      const char* fragmentPath
    );
    Shader(
      const char* vertexPath, 
      const char* geometryPath,
      const char* fragmentPath
    );
    ~Shader();

    // Use/activate the shader and return ID
    unsigned int use();

    // Delete the shader and return ID
    unsigned int del();

    // Utility uniform functions
    void setBool(const std::string &name, bool value) const;
    void setInt(const std::string &name, int value) const;
    void setFloat(const std::string &name, float value) const;
    void setVec3(const std::string &name, glm::vec3 value) const;
    void setMat4(const std::string &name, glm::mat4 value) const;
};