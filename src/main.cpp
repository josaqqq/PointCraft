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
#include "modes.hpp"
#include "octree.hpp"
#include "constants.hpp"

//
// Point cloud info
//
Eigen::MatrixXd meshV;    // double matrix of vertex positions
Eigen::MatrixXd meshTC;   // double matrix of texture coordinates
Eigen::MatrixXd meshN;    // double matrix of corner normals
Eigen::MatrixXi meshF;    // list of face indices into vertex positions
Eigen::MatrixXi meshFTC;  // list of face indices into vertex texture coordinates
Eigen::MatrixXi meshFN;   // list of face indices into vertex normals

double averageDistance;

//
// Mode Selection
//
ModeSelector modeSelector;

void callback() {
  ImGuiIO &io = ImGui::GetIO();

  ImGui::PushItemWidth(100);

  // Surface Reconstruction
  ImGui::Text("Surface Reconstruction:");
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Poisson Surface Reconstruction"))  poissonReconstruct(meshV, meshN);
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Moving Least Squares"))            mlsSmoothing(meshV);
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Greedy Projection Triangulation")) greedyProjection(meshV, meshN);

  // Enable mode selection
  modeSelector.enableModeSelection(io);

  ImGui::PopItemWidth();
}

void movePointsToOrigin(Eigen::MatrixXd &meshV) {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  for (int i = 0; i < meshV.rows(); i++) {
    x += meshV(i, 0);
    y += meshV(i, 1);
    z += meshV(i, 2);
  }
  x /= static_cast<double>(meshV.rows());
  y /= static_cast<double>(meshV.rows());
  z /= static_cast<double>(meshV.rows());

  for (int i = 0; i < meshV.rows(); i++) {
    meshV(i, 0) -= x;
    meshV(i, 1) -= y;
    meshV(i, 2) -= z;
  }
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
  polyscope::options::autocenterStructures = false;
  polyscope::options::autoscaleStructures = false;
  polyscope::view::windowWidth = WindowWidth;
  polyscope::view::windowHeight = WindowHeight;

  // Initialize polyscope
  polyscope::init();
  polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
  polyscope::view::bgColor = BackgroundColor;

  std::string filename = args::get(inFile);
  std::cout << "loading: " << filename << std::endl;

  // Read the mesh
  igl::readOBJ(filename, meshV, meshTC, meshN, meshF, meshFTC, meshFN);
  std::cout << "Vertex num:\t\t"            << meshV.rows()   << std::endl;
  std::cout << "Texture coordinate num:\t"  << meshTC.rows()  << std::endl;
  std::cout << "Normal num:\t\t"            << meshN.rows()   << std::endl;
  std::cout << "Face num:\t\t"              << meshF.rows()   << std::endl;
  if (meshN.rows() == 0) std::cout << "ERROR: Please include normal information." << std::endl;

  // Move points to set the gravity point to (0.0, 0.0, 0.0).
  movePointsToOrigin(meshV);

  // Calculate the average distance using octree.
  Octree octree(meshV, OctreeResolution);
  averageDistance = octree.calcAverageDistance();
  std::cout << "Average Distance:\t" << averageDistance << std::endl;

  // Register Points
  polyscope::PointCloud* pointCloud = polyscope::registerPointCloud(PointName, meshV);
  pointCloud->setPointColor(PointColor);
  pointCloud->setPointRadius(PointRadius);

  // Register Normals
  polyscope::PointCloudVectorQuantity *vectorQuantity = pointCloud->addVectorQuantity(NormalName, meshN);
  vectorQuantity->setVectorColor(NormalColor);
  vectorQuantity->setVectorLengthScale(NormalLength);
  vectorQuantity->setVectorRadius(NormalRadius);
  vectorQuantity->setEnabled(NormalEnabled);
  vectorQuantity->setMaterial(NormalMaterial);

  // Reconstruct surface
  poissonReconstruct(meshV, meshN);
  greedyProjection(meshV, meshN);

  // Add the callback
  polyscope::state::userCallback = callback;

  // Show the gui
  polyscope::show();

  return 0;
}
