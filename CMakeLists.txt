cmake_minimum_required(VERSION 3.12)

project(PointCraft)
set(PROJECT_VERSION 1.0.0)

# Maybe stop from CMAKEing in the wrong place
if (CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR "Source and build directories cannot be the same. Go use the /build directory.")
endif()

### Configure output locations
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

### Compiler options
# Emit a compile flags file to support completion engines 
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  # using Clang (linux or apple) or GCC
  message("Using clang/gcc compiler flags")
  SET(BASE_CXX_FLAGS "-std=c++17 -Wall -Wextra -g3")
  SET(DISABLED_WARNINGS " -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function -Wno-deprecated-declarations -Wno-missing-braces")
  SET(TRACE_INCLUDES " -H -Wno-error=unused-command-line-argument")

  if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    message("Setting clang-specific options")
    SET(BASE_CXX_FLAGS "${BASE_CXX_FLAGS} -ferror-limit=5 -fcolor-diagnostics")
    SET(CMAKE_CXX_FLAGS_DEBUG          "-fsanitize=address -fno-limit-debug-info")
  elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    SET(BASE_CXX_FLAGS "${BASE_CXX_FLAGS} -fmax-errors=5")
    message("Setting gcc-specific options")
    SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} -Wno-maybe-uninitialized -Wno-format-zero-length -Wno-unused-but-set-parameter -Wno-unused-but-set-variable")
  endif()

  SET(CMAKE_CXX_FLAGS "${BASE_CXX_FLAGS} ${DISABLED_WARNINGS}")
  #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TRACE_INCLUDES}") # uncomment if you need to track down where something is getting included from
  SET(CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG} -g3")
  SET(CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
  SET(CMAKE_CXX_FLAGS_RELEASE        "-march=native -O3 -DNDEBUG")
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")
  
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  # using Visual Studio C++
  message("Using Visual Studio compiler flags")
  set(BASE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  set(BASE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP") # parallel build
  SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} /wd\"4267\"")  # ignore conversion to smaller type (fires more aggressively than the gcc version, which is annoying)
  SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} /wd\"4244\"")  # ignore conversion to smaller type (fires more aggressively than the gcc version, which is annoying)
  SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} /wd\"4305\"")  # ignore truncation on initialization
  SET(CMAKE_CXX_FLAGS "${BASE_CXX_FLAGS} ${DISABLED_WARNINGS}")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd")

  add_definitions(/D "_CRT_SECURE_NO_WARNINGS")
  add_definitions(-DNOMINMAX)
  add_definitions(-D_USE_MATH_DEFINES)

else()
  # unrecognized
  message( FATAL_ERROR "Unrecognized compiler [${CMAKE_CXX_COMPILER_ID}]" )
endif()

# Find Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Find PCL
find_package(PCL 1.10 REQUIRED)

# Add polyscope
add_subdirectory("deps/polyscope")

# Create an executable
add_executable(
    PointCraft
    src/main.cpp
    src/surface.cpp
    src/ray.cpp
    src/plane.cpp
    src/cluster.cpp
    src/gui.cpp
    src/point_cloud.cpp
    src/sketch_tool.cpp
    src/sketch_interpolation_tool.cpp
    src/spray_interpolation_tool.cpp
    src/feature_tool.cpp
    src/delete_tool.cpp
)

# Include settings
# Specify the directories to be searched for included files (headers).
# Use libigl as libIGL in "header only" mode.
target_include_directories(PointCraft PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/deps/libigl/include")
target_include_directories(PointCraft PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/deps/polyscope/deps/args/")
target_include_directories(PointCraft PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/deps/polyscope/deps/json/include")

# Link settings
# Specify the libraries to be linked to your target (executable or library)
target_link_libraries(PointCraft polyscope ${EIGEN_LIBRAIRES} ${PCL_LIBRARIES})

message(STATUS "--------------------------------------------")
message(STATUS "Info :")
message(STATUS "  Project Name =            ${PROJECT_NAME}")
message(STATUS "  Version =                 ${PROJECT_VERSION}")
message(STATUS "  CMAKE_VERSION =           ${CMAKE_VERSION}")
message(STATUS "  CMAKE_INSTALL_PREFIX =    ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  CMAKE_INSTALL_LIBDIR =    ${CMAKE_INSTALL_LIBDIR}")
message(STATUS "  EIGEN_VERSION =           ${EIGEN_VERSION}")
message(STATUS "  PCL_VERSION =             ${PCL_VERSION}")
