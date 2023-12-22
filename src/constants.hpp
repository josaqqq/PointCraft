#pragma once

//
// Screen
//
const std::array<float, 4> BackgroundColor = { 0.025, 0.025, 0.025, 0.000 };
const int WindowWidth = 1024;
const int WindowHeight = 1024;
const double ScreenOffset = 10.0;

//
// Point Cloud
//
const double PointCloudBoundingBoxSide = 1.0d;
const double PointCloudDownsampleVoxel = 0.010d;

const std::string PointName = "Point Cloud";
const glm::dvec3 PointColor = { 1.000, 1.000, 1.000 };
const double PointRadius = 0.002;

const std::string NormalName = "normal vector";
const glm::dvec3 NormalColor =  { 1.000, 1.000, 1.000 };
const double NormalLength = 0.015;
const double NormalRadius = 0.001;
const bool NormalEnabled = true;
const std::string NormalMaterial = "flat";

//
// Octree parameters
//
const double OctreeResolution = 0.010;  // The length of the smallest voxels at lowest octree level.

//
// Surface points
//
const std::string SurfacePointName = "Surface Points";

const glm::dvec3 SurfacePointColor = { 0.000, 1.000, 0.000 };
const double SurfacePointRadius = 0.0025;
const std::string SurfaceMaterial = "flat";

//
// Sketch
//
const std::string SketchPointName = "Sketch Points";

const glm::dvec3 SketchColor = { 0.000, 1.000, 0.000 };
const double SketchRadius  = 0.00020;
const std::string SketchMaterial = "flat";

const glm::dvec3 BasisPointColor = { 0.000, 1.000, 0.000 };
const double BasisPointRadius = 0.0025;
const double BasisNormalLength = 0.05;
const double BasisNormalRadius = 0.003;

const std::string SketchPrefix = "Sketch: ";

//
// Tools
//
const int SketchSurfacePointNum = 10;
const int SpraySurfacePointNum = 30;
const int DeleteSurfacePointNum = 5;

//
// Poisson Surface Reconstruction
//
const int PoissonMaxDepth = 6;
const int PoissonThreadNum = 4;

const std::string PoissonName = "Poisson Surface Reconstruction";
const glm::dvec3 PoissonColor = { 0.000, 0.196, 1.000 };
const glm::dvec3 PoissonBackFaceColor = { 1.000, 0.686, 0.000 };
const std::string PoissonMaterial = "clay";
const polyscope::BackFacePolicy PoissonBackFacePolicy = polyscope::BackFacePolicy::Custom;

//
// Moving Least Squares
//
const int MLS_SpraySize = 5;
const int MLSPolynomialOrder = 2;

const std::string MLSName = "Interpolation MLS";
const glm::dvec3 MLSColor = { 0.000, 0.000, 1.000 };

//
// Greedy Projection
const std::string GreedyProjName = "Greedy Projection Triangulation";

const double GreedyProjSearchRadiusMult = 5.0;
const double GreedyProjHoleDetectMult = 10.0;

const double GreedyProjMu = 2.5;                   
const int GreedyProjMaxNN = 100;
const double GreedyProjMaxSurfaceAngle = 9.0*M_PI/18.0;   // 90 degrees
const double GreedyProjMinAngle = M_PI/18.0;              // 10 degrees
const double GreedyProjMaxAngle = 17.0*M_PI/18.0;         // 170 degrees
const bool GreedyProjNormalConsistency = true;
const bool GreedyProjVertexConsistency = true;

const glm::dvec3 GreedyProjColor = { 0.000, 0.196, 1.000 };
const glm::dvec3 GreedyProjBackFaceColor = { 1.000, 0.686, 0.000 };
const glm::dvec3 GreedyProjBoundaryColor = { 1.000, 0.039, 0.039 };
const std::string GreedyProjMaterial = "clay";
const polyscope::BackFacePolicy GreedyProjBackFacePolicy = polyscope::BackFacePolicy::Custom;

//
// Pseudo Surface
//
const std::string PseudoSurfaceName = "Pseudo Surface";
const std::string TemporalPseudoSurfaceName = "Pseudo Surface (Temporal)";
const glm::dvec3 PseudoSurfaceColor =  { 0.000, 0.196, 1.000 };
const glm::dvec3 PseudoSurfaceBackFaceColor = { 1.000, 0.686, 0.000 };
const glm::dvec3 PseudoSurfaceBoundaryColor = { 1.000, 0.039, 0.039 };
const std::string PseudoSurfaceMaterial = "clay";
const polyscope::BackFacePolicy PseudoSurfaceBackFacePolicy = polyscope::BackFacePolicy::Custom;

//
// Clustering
//
const double DBSCAN_SearchRange = 1.0;
const int DBSCAN_MinPoints = 1;

const std::string DBSCAN_Name = "DBSCAN Labeling: ";
const bool DBSCAN_Enabled = false;