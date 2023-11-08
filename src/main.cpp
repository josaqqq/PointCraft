#include "polyscope/polyscope.h"

#include <igl/PI.h>
#include <igl/avg_edge_length.h>
#include <igl/barycenter.h>
#include <igl/boundary_loop.h>
#include <igl/exact_geodesic.h>
#include <igl/gaussian_curvature.h>
#include <igl/invert_diag.h>
#include <igl/lscm.h>
#include <igl/massmatrix.h>
#include <igl/per_vertex_normals.h>
#include <igl/readOBJ.h>
#include <igl/readPLY.h>

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

Eigen::MatrixXd meshV;    // double matrix of vertex positions
Eigen::MatrixXd meshTC;   // double matrix of texture coordinates
Eigen::MatrixXd meshN;    // double matrix of corner normals
Eigen::MatrixXi meshF;    // list of face indices into vertex positions
Eigen::MatrixXi meshFTC;  // list of face indices into vertex texture coordinates
Eigen::MatrixXi meshFN;   // list of face indices into vertex normals

// Options for algorithms
int iVertexSource = 7;

// for Patch Mode
bool DraggingMode = false;
int PatchNum = 0;
std::vector<std::vector<double>> Patch;

void addCurvatureScalar() {
  using namespace Eigen;
  using namespace std;

  VectorXd K;
  igl::gaussian_curvature(meshV, meshF, K);
  SparseMatrix<double> M, Minv;
  igl::massmatrix(meshV, meshF, igl::MASSMATRIX_TYPE_DEFAULT, M);
  igl::invert_diag(M, Minv);
  K = (Minv * K).eval();

  polyscope::getSurfaceMesh("input mesh")
      ->addVertexScalarQuantity("gaussian curvature", K,
                                polyscope::DataType::SYMMETRIC);
}

void computeDistanceFrom() {
  Eigen::VectorXi VS, FS, VT, FT;
  // The selected vertex is the source
  VS.resize(1);
  VS << iVertexSource;
  // All vertices are the targets
  VT.setLinSpaced(meshV.rows(), 0, meshV.rows() - 1);
  Eigen::VectorXd d;
  igl::exact_geodesic(meshV, meshF, VS, FS, VT, FT, d);

  polyscope::getSurfaceMesh("input mesh")
      ->addVertexDistanceQuantity(
          "distance from vertex " + std::to_string(iVertexSource), d);
}

void computeParameterization() {
  using namespace Eigen;
  using namespace std;

  // Fix two points on the boundary
  VectorXi bnd, b(2, 1);
  igl::boundary_loop(meshF, bnd);

  if (bnd.size() == 0) {
    polyscope::warning("mesh has no boundary, cannot parameterize");
    return;
  }

  b(0) = bnd(0);
  b(1) = bnd((int)round(bnd.size() / 2));
  MatrixXd bc(2, 2);
  bc << 0, 0, 1, 0;

  // LSCM parametrization
  Eigen::MatrixXd V_uv;
  igl::lscm(meshV, meshF, b, bc, V_uv);

  polyscope::getSurfaceMesh("input mesh")
      ->addVertexParameterizationQuantity("LSCM parameterization", V_uv);
}

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

  static int numPoints = meshV.rows();
  static float param = 3.14;

  // Tutorial window
  {
    ImGui::PushItemWidth(100);

    ImGui::InputInt("num points", &numPoints);
    ImGui::InputFloat("param value", &param);

    // Curvature
    if (ImGui::Button("add curvature")) {
      addCurvatureScalar();
    }
    
    // Normals 
    if (ImGui::Button("add normals")) {
      computeNormals();
    }

    // Param
    if (ImGui::Button("add parameterization")) {
      computeParameterization();
    }

    // Geodesics
    if (ImGui::Button("compute distance")) {
      computeDistanceFrom();
    }
    ImGui::SameLine();
    ImGui::InputInt("source vertex", &iVertexSource);

    if (ImGui::Button("hello world!")) {
      std::cout << "hello world!" << std::endl;
    }

    bool isHovered = ImGui::IsItemHovered();
    bool isFocused = ImGui::IsItemFocused();
    ImVec2 mousePositionAbsolute = ImGui::GetMousePos();
    ImVec2 screenPositionAbsolute = ImGui::GetItemRectMin();
    ImVec2 mousePositionRelative = ImVec2(mousePositionAbsolute.x - screenPositionAbsolute.x, mousePositionAbsolute.y - screenPositionAbsolute.y);
    ImGui::Text("Is mouse over screen? %s", isHovered ? "Yes" : "No");
    ImGui::Text("Is screen focused? %s", isFocused ? "Yes" : "No");
    ImGui::Text("Position: %f, %f", mousePositionRelative.x, mousePositionRelative.y);
    ImGui::Text("Mouse clicked: %s", ImGui::IsMouseDown(ImGuiMouseButton_Left) ? "Yes" : "No");

    if (ImGui::Checkbox("patch mode", &DraggingMode)) {
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
  polyscope::PointCloud* pointCloud = polyscope::registerPointCloud("point cloud", meshV);
  pointCloud->setPointRadius(0.001);
  pointCloud->setPointColor({ 0.142, 0.448, 1.000 });

  polyscope::PointCloudVectorQuantity *vectorQuantity = pointCloud->addVectorQuantity("normal vector", meshN);
  vectorQuantity->setVectorLengthScale(0.015);
  vectorQuantity->setVectorRadius(0.001);
  vectorQuantity->setVectorColor({ 0.110, 0.388, 0.890 });
  vectorQuantity->setMaterial("normal");

  // Add the callback
  polyscope::state::userCallback = callback;

  // Show the gui
  polyscope::show();

  return 0;
}
