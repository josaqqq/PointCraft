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
  polyscope::view::windowWidth = WindowWidth;
  polyscope::view::windowHeight = WindowHeight;
  polyscope::view::bgColor = BackgroundColor;

  polyscope::state::lengthScale = 1.0;

  polyscope::options::autocenterStructures = false;
  polyscope::options::autoscaleStructures = false;
  polyscope::options::automaticallyComputeSceneExtents = false;
  polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
  // polyscope::options::buildDefaultGuiPanels = false;

  // Initialize polyscope
  polyscope::init();

  // Initialize classes
  int currentMode;
  int currentSurfaceMode;

  PointCloud pointCloud(args::get(inFile));

  InterpolationTool interpolationTool(&currentMode, &pointCloud);
  MLSSprayTool      mlsSprayTool(&currentMode, &pointCloud);
  DeleteTool        deleteTool(&currentMode, &pointCloud);

  modeSelector = ModeSelector(
    &currentMode, 
    &currentSurfaceMode, 

    &pointCloud, 
    
    &interpolationTool,
    &mlsSprayTool,
    &deleteTool
  );

  // Reconstruct Surfaces
  Surface poissonSurface(PoissonName, pointCloud.getVertices(), pointCloud.getNormals());
  poissonSurface.reconstructPoissonSurface(pointCloud.getAverageDistance());

  // Add the callback
  polyscope::state::userCallback = callback;

  // Show the gui
  polyscope::show();

  return 0;
}
