#include "polyscope/polyscope.h"

#include <igl/per_vertex_normals.h>
#include <igl/readOBJ.h>

#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/pick.h"
#include "polyscope/view.h"

#include <iostream>
#include <random>
#include <string>

#include "args/args.hxx"
#include "json/json.hpp"

#include "surface.hpp"

Eigen::MatrixXd meshV;    // double matrix of vertex positions
Eigen::MatrixXd meshTC;   // double matrix of texture coordinates
Eigen::MatrixXd meshN;    // double matrix of corner normals
Eigen::MatrixXi meshF;    // list of face indices into vertex positions
Eigen::MatrixXi meshFTC;  // list of face indices into vertex texture coordinates
Eigen::MatrixXi meshFN;   // list of face indices into vertex normals

// for Patch Mode
bool DraggingMode = false;
int PatchNum = 0;
std::vector<std::vector<double>> Patch;

void computeNormals() {
  Eigen::MatrixXd N_vertices;
  igl::per_vertex_normals(meshV, meshF, N_vertices);

  polyscope::getSurfaceMesh("input mesh")
      ->addVertexVectorQuantity("libIGL vertex normals", N_vertices);
}

void addPatchToPointCloud() {
  polyscope::PointCloud* patch = polyscope::registerPointCloud("patch " + std::to_string(PatchNum), Patch);
  patch->setPointRadius(0.002);
  patch->setPointColor({ 0.890, 0.110, 0.778 });

  PatchNum++;
  Patch.clear();
}

void callback() {
  ImGuiIO &io = ImGui::GetIO();

  ImGui::PushItemWidth(100);

  // Surface
  if (ImGui::Button("Surface Reconstruction")) {
    surfaceReconstruct(meshV, meshN);
  }

  // Normals 
  if (ImGui::Button("Add Normals")) {
    computeNormals();
  }

  if (ImGui::Checkbox("Patch Mode", &DraggingMode)) {
    if (DraggingMode) {
      // 0 -> 1: positive edge
      polyscope::view::moveScale = 0.0;
    } else {
      // 1 -> 0: negative edge
      polyscope::view::moveScale = 2.0;
    }
  }

  // Press or Release
  if (DraggingMode) {
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
      ImVec2 mousePos = ImGui::GetMousePos();
      int xPos = io.DisplayFramebufferScale.x * mousePos.x;
      int yPos = io.DisplayFramebufferScale.y * mousePos.y;
      std::pair<polyscope::Structure*, size_t> pickResult = polyscope::pick::evaluatePickQuery(xPos, yPos);

      if (pickResult.first != nullptr) {
        polyscope::PointCloud* pointCloud = polyscope::getPointCloud(pickResult.first->name);
        if (pointCloud != nullptr) {
          glm::vec3 pointPos = pointCloud->getPointPosition(pickResult.second);
          std::cout << pointPos.x << " " << pointPos.y << " " << pointPos.z << std::endl;
          Patch.push_back({
            pointPos.x, pointPos.y, pointPos.z
          });
        }
      }
    } 
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && Patch.size() > 1) {
      // TODO: Guard for dragging on windows. checkbox is sufficient?
      addPatchToPointCloud();
    }
  }

  // Point Selection

  ImGui::PopItemWidth();
}

int main(int argc, char **argv) {
  // Configure the argument parser
  // -h or --help
  args::ArgumentParser parser("Point cloud editor tool"
                              "");
  // args information
  args::Positional<std::string> inFile(parser, "mesh", "input mesh");

  // Parse args
  try {
    parser.ParseCLI(argc, argv);
  } catch (args::Help) {
    std::cout << parser;
    return 0;
  } catch (args::ParseError e) {
    std::cerr << e.what() << std::endl;

    std::cerr << parser;
    return 1;
  }

  // Options
  polyscope::view::windowWidth = 1024;
  polyscope::view::windowHeight = 1024;

  // Initialize polyscope
  polyscope::init();
  polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
  polyscope::view::bgColor = { 0.025, 0.025, 0.025, 0.000 };

  std::string filename = args::get(inFile);
  std::cout << "loading: " << filename << std::endl;

  // Read the mesh
  igl::readOBJ(filename, meshV, meshTC, meshN, meshF, meshFTC, meshFN);
  std::cout << "Vertex num:\t" << meshV.rows() << std::endl;
  std::cout << "Texture coordinate num:\t" << meshTC.rows() << std::endl;
  std::cout << "Normal num:\t" << meshN.rows() << std::endl;
  std::cout << "Face num:\t" << meshF.rows() << std::endl;

  // Visualize point cloud
  polyscope::PointCloud* pointCloud = polyscope::registerPointCloud("Point Cloud", meshV);
  pointCloud->setPointRadius(0.002);
  pointCloud->setPointColor({ 0.142, 0.448, 1.000 });

  polyscope::PointCloudVectorQuantity *vectorQuantity = pointCloud->addVectorQuantity("normal vector", meshN);
  vectorQuantity->setVectorLengthScale(0.015);
  vectorQuantity->setVectorRadius(0.001);
  vectorQuantity->setVectorColor({ 0.110, 0.388, 0.890 });
  // vectorQuantity->setMaterial("normal");

  // Add the callback
  polyscope::state::userCallback = callback;

  // Show the gui
  polyscope::show();

  return 0;
}
