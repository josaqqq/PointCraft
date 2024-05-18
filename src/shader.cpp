#include "shader.hpp"

Shader::Shader(
  const char* vertexPath, 
  const char* fragmentPath
) {
  // Retrieve the vertex/fragment source code from filePath
  std::string vertexCode;
  std::string fragmentCode;

  std::ifstream vShaderFile;
  std::ifstream fShaderFile;
  vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
  fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);

  try {
    // Open files
    vShaderFile.open(vertexPath);
    fShaderFile.open(fragmentPath);

    // Read file's buffer contents into streams
    std::stringstream vShaderStream, fShaderStream;
    vShaderStream << vShaderFile.rdbuf();
    fShaderStream << fShaderFile.rdbuf();

    // Close file handlers
    vShaderFile.close();
    fShaderFile.close();

    // Convert stream into string
    vertexCode = vShaderStream.str();
    fragmentCode = fShaderStream.str();
  } catch (std::ifstream::failure e) {
    std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
  }
  const char* vShaderCode = vertexCode.c_str();
  const char* fShaderCode = fragmentCode.c_str();

  // Compile shaders
  unsigned int vertex, fragment;
  int success;
  char infoLog[512];

  // Vertex Shader
  vertex = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex, 1, &vShaderCode, NULL);
  glCompileShader(vertex);
  glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(vertex, 512, NULL, infoLog);
    std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
  }

  // Fragment Shader
  fragment = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragment, 1, &fShaderCode, NULL);
  glCompileShader(fragment);
  glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(fragment, 512, NULL, infoLog);
    std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
  }

  // Shader program
  ID = glCreateProgram();
  glAttachShader(ID, vertex);
  glAttachShader(ID, fragment);
  glLinkProgram(ID);
  glGetProgramiv(ID, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(ID, 512, NULL, infoLog);
    std::cout << "ERROR::PROGRAM::COMPILATION_FAILED\n" << infoLog << std::endl;
  }
  glDeleteShader(vertex);
  glDeleteShader(fragment);
}

Shader::Shader(
  const char* vertexPath, 
  const char* geometryPath,
  const char* fragmentPath
) {
  // Retrieve the vertex/geometry/fragment source code from filePath
  std::string vertexCode;
  std::string geometryCode;
  std::string fragmentCode;

  std::ifstream vShaderFile;
  std::ifstream gShaderFile;
  std::ifstream fShaderFile;
  vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
  gShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
  fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);

  try {
    // Open files
    vShaderFile.open(vertexPath);
    gShaderFile.open(geometryPath);
    fShaderFile.open(fragmentPath);

    // Read file's buffer contents into streams
    std::stringstream vShaderStream, gShaderStream, fShaderStream;
    vShaderStream << vShaderFile.rdbuf();
    gShaderStream << gShaderFile.rdbuf();
    fShaderStream << fShaderFile.rdbuf();

    // Close file handlers
    vShaderFile.close();
    gShaderFile.close();
    fShaderFile.close();

    // Convert stream into string
    vertexCode = vShaderStream.str();
    geometryCode = gShaderStream.str();
    fragmentCode = fShaderStream.str();
  } catch (std::ifstream::failure e) {
    std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
  }
  const char* vShaderCode = vertexCode.c_str();
  const char* gShaderCode = geometryCode.c_str();
  const char* fShaderCode = fragmentCode.c_str();

  // Compile shaders
  unsigned int vertex, geometry, fragment;
  int success;
  char infoLog[512];

  // Vertex Shader
  vertex = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex, 1, &vShaderCode, NULL);
  glCompileShader(vertex);
  glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(vertex, 512, NULL, infoLog);
    std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
  }

  // Geometry Shader
  geometry = glCreateShader(GL_GEOMETRY_SHADER);
  glShaderSource(geometry, 1, &gShaderCode, NULL);
  glCompileShader(geometry);
  glGetShaderiv(geometry, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(geometry, 512, NULL, infoLog);
    std::cout << "ERROR::SHADER::GEOMETRY::COMPILATION_FAILED\n" << infoLog << std::endl;
  }

  // Fragment Shader
  fragment = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragment, 1, &fShaderCode, NULL);
  glCompileShader(fragment);
  glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(fragment, 512, NULL, infoLog);
    std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
  }

  // Shader program
  ID = glCreateProgram();
  glAttachShader(ID, vertex);
  glAttachShader(ID, geometry);
  glAttachShader(ID, fragment);
  glLinkProgram(ID);
  glGetProgramiv(ID, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(ID, 512, NULL, infoLog);
    std::cout << "ERROR::PROGRAM::COMPILATION_FAILED\n" << infoLog << std::endl;
  }
  glDeleteShader(vertex);
  glDeleteShader(geometry);
  glDeleteShader(fragment);
}

Shader::~Shader() {
  del();
  std::cout << "Shader " << ID << " was released!" << std::endl;
}

unsigned int Shader::use() {
  glUseProgram(ID);
  return ID;
}
unsigned int Shader::del() {
  glDeleteProgram(ID);
  return ID;
}

void Shader::setBool(const std::string &name, bool value) const {
  glUniform1i(glGetUniformLocation(ID, name.c_str()), (int)value);
}
void Shader::setInt(const std::string &name, int value) const {
  glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
}
void Shader::setFloat(const std::string &name, float value) const {
  glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
}
void Shader::setVec3(const std::string &name, glm::vec3 value) const {
  glUniform3f(glGetUniformLocation(ID, name.c_str()), value.x, value.y, value.z);
}
void Shader::setMat4(const std::string &name, glm::mat4 value) const {
  glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, glm::value_ptr(value));
}