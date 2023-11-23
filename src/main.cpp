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

ModeSelector modeSelector;

void callback() {
  ImGuiIO &io = ImGui::GetIO();

  ImGui::PushItemWidth(50);

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
  // polyscope::options::buildDefaultGuiPanels = false;
  polyscope::view::bgColor = BackgroundColor;

  // Initialize classes
  int currentMode;
  int currentPointCloud;
  int currentPointCloudNormal;
  int currentSurfaceMode;
  PointCloud pointCloud(args::get(inFile));
  InterpolationTool interpolationTool(&pointCloud, &currentMode);
  modeSelector = ModeSelector(
    &currentMode, 
    &currentPointCloud,
    &currentPointCloudNormal,
    &currentSurfaceMode, 
    &pointCloud, 
    &interpolationTool
  );

  // Reconstruct Surfaces
  poissonReconstruct(PoissonName, pointCloud.averageDistance, pointCloud.Vertices, pointCloud.Normals);
  greedyProjection(GreedyProjName, pointCloud.Vertices, pointCloud.Normals);

  // Add the callback
  polyscope::state::userCallback = callback;

  // Show the gui
  polyscope::show();

  return 0;
}
