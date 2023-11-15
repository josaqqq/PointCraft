#include "polyscope/polyscope.h"

#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/view.h"

#include <iostream>
#include <random>
#include <string>

#include "args/args.hxx"
#include "json/json.hpp"

#include "surface.hpp"
#include "modes.hpp"
#include "point_cloud.hpp"
#include "constants.hpp"

int currentMode;
int currentSurfaceMode;
PointCloud pointCloud;
ModeSelector modeSelector;

void callback() {
  ImGuiIO &io = ImGui::GetIO();

  ImGui::PushItemWidth(100);

  // Surface Reconstruction
  ImGui::Text("Surface Reconstruction:");
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Poisson Surface Reconstruction"))  poissonReconstruct(pointCloud.meshV, pointCloud.meshN);
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Moving Least Squares"))            mlsSmoothing(pointCloud.meshV);
  ImGui::Text("   "); ImGui::SameLine();
  if (ImGui::Button("Greedy Projection Triangulation")) greedyProjection(pointCloud.meshV, pointCloud.meshN);

  // Enable mode selection
  modeSelector.enableModeSelection(io);

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
  polyscope::options::autocenterStructures = false;
  polyscope::options::autoscaleStructures = false;
  polyscope::view::windowWidth = WindowWidth;
  polyscope::view::windowHeight = WindowHeight;

  // Initialize polyscope
  polyscope::init();
  polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
  polyscope::view::bgColor = BackgroundColor;

  // Initialize classes
  pointCloud = PointCloud(args::get(inFile));
  InterpolationTool interpolationTool(&pointCloud, &currentMode);
  modeSelector = ModeSelector(&currentMode, &currentSurfaceMode, &pointCloud, &interpolationTool);

  // Reconstruct surface
  poissonReconstruct(pointCloud.meshV, pointCloud.meshN);
  greedyProjection(pointCloud.meshV, pointCloud.meshN);

  // Add the callback
  polyscope::state::userCallback = callback;

  // Show the gui
  polyscope::show();

  return 0;
}
