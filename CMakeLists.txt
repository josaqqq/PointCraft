cmake_minimum_required(VERSION 3.12)

project(PointCraft)

find_package(glfw3 REQUIRED)

include_directories(include/glm)
include_directories(include/glad/include)
add_subdirectory(include/glad)

add_compile_options(-O2 -Wall)

add_executable(PointCraft ./src/main.cpp)

target_link_libraries(PointCraft glad glfw ${CMAKE_DL_LIBS})

set_property(TARGET PointCraft PROPERTY CXX_STANDARD 17)
