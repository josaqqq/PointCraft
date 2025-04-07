# PointCraft
<div align="center">
<img width="421" alt="image" src="https://github.com/sakajo6/PointCraft/assets/68010363/e44dc25e-1f69-44ac-84b3-2732bbc83735">
</div>

PointCraft is an innovative user interface for point cloud editing. It allows users to interactively edit point cloud data using a sketch-based interface. Users can add new points to fill missing areas or erase unnecessary points with intuitive tools.

## Features

- **Sketch Tool**: Fill missing areas in a point cloud by sketching boundaries. The system automatically detects the intended depth and interpolates the missing regions with new points.
- **Delete Tool**: Remove unwanted points by sketching boundaries around unnecessary areas. The system erases all points within the sketched region.
- **Real-Time Rendering**: Visualize point cloud data in real-time using OpenGL.
- **Undo/Redo Support**: Easily revert or reapply changes during editing.

## Installation

To build and run PointCraft, follow these steps:

1. Clone the repository and navigate to the `PointCraft` directory.
2. Install the required dependencies:
   - [GLFW](https://www.glfw.org/)
   - [GLAD](https://glad.dav1d.de/)
   - [Dear ImGui](https://github.com/ocornut/imgui)
   - [PCL (Point Cloud Library)](https://pointclouds.org/)
   - [GLM (OpenGL Mathematics)](https://github.com/g-truc/glm)
3. Build the project using CMake:

   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

4. Run the application:

```
./PointCloudEditor
```

## Usage
1. Sketch Tool: Use the sketch tool to outline missing areas in the point cloud. The system will interpolate and fill the region with new points.
2. Delete Tool: Use the delete tool to outline unwanted areas. The system will remove all points within the sketched boundary.

## Directory Structure
```
PointCraft/
├── include/          # Header files for core functionality
├── src/              # Source files for the application
├── shader/           # Shader programs for rendering
├── data/             # Sample point cloud data
├── CMakeLists.txt    # Build configuration
└── [README.md](http://_vscodecontentref_/0)         # Project documentation
```
