#pragma once

//
// Scene parameters
//
const std::array<float, 4> BackgroundColor = { 0.025, 0.025, 0.025, 0.000 };
const int WindowWidth = 1024;
const int WindowHeight = 1024;
const double ScreenOffset = 10.0;

//
// Point Cloud parameters
//
const std::string PointName = "Point Cloud";
const glm::dvec3 PointColor = { 1.000, 1.000, 1.000 };
const double PointRadius = 0.002;

const std::string NormalName = "normal vector";
const glm::dvec3 NormalColor =  { 1.000, 1.000, 1.000 };
const double NormalLength = 0.015;
const double NormalRadius = 0.001;
const bool NormalEnabled = true;
const std::string NormalMaterial = "flat";

const std::string ScalarName = "depth map";
const std::string ScalarColorMap = "viridis";
const bool ScalarEnabled = true;

const std::string SurfacePointName = "Surface Points";
const int SurfacePointNum = 10;

//
// Surface points
//
const glm::dvec3 SurfacePointColor = { 0.000, 1.000, 0.000 };
const double SurfacePointSize = 0.005;
const std::string SurfaceMaterial = "flat";

//
// Octree parameters
//
const double OctreeResolution = 0.01;  // The length of the smallest voxels at lowest octree level.
const int RayMaxStep = 10000;

//
// Patch parameters
//
const int PatchSize = 50;
const double depthInterval = 5.0; // Multiple with averageDistance of input point cloud.

const glm::dvec3 DiscretizedPointColor = { 1.000, 0.000, 0.000 };
const double DiscretizedPointRadius = 0.0025;

const std::string SketchPointName = "Sketch Points";
const glm::dvec3 SketchPointColor = { 0.000, 0.000, 1.000 };

const glm::dvec3 BasisPointColor = { 0.000, 1.000, 0.000 };
const double BasisPointRadius = 0.0025;
const double BasisNormalLength = 0.05;
const double BasisNormalRadius = 0.003;

const std::string TracePrefix = "Trace: ";
const std::string CastPrefix = "Cast: ";
const std::string SketchPrefix = "Sketch: ";

const int MLS_SprayNearestNeighbors = 30;
const int MLS_SpraySize = 4;

//
//  Curve Network
//
const glm::dvec3 CurveNetworkColor = { 0.000, 1.000, 0.000 };
const double CurveNetworkRadius   = 0.00025;
const std::string CurveNetworkMaterial = "flat";

//
// Poisson Surface Reconstruction
//
const std::string PoissonName = "Poisson Surface Reconstruction";
const int PoissonMaxDepth = 5;
const int PoissonThreadNum = 4;
const glm::dvec3 PoissonColor = { 0.155, 0.186, 0.790 };
const std::string PoissonMaterial = "normal";
const bool PoissonEnabled = false;

//
// Radial Basis Functions
//
const std::string RBFName = "Interpolation: RBF";
const glm::dvec3 RBFColor = { 1.000, 0.000, 0.000 };

//
// Moving Least Squares
//
const std::string MLSName = "Interpolation MLS";
const glm::dvec3 MLSColor = { 0.000, 0.000, 1.000 };

const int MLSPolynomialOrder = 2;

//
// Delaunay Triangulation
//
const std::string GreedyProjName = "Greedy Projection Triangulation";
const double GreedyProjSearchRadius = 0.5;
const double GreedyProjMu = 2.5;
const int GreedyProjMaxNN = 100;
const double GreedyProjMaxSurfaceAngle = M_PI/4.0; // 45 degrees
const double GreedyProjMinAngle = M_PI/18.0;       // 10 degrees
const double GreedyProjMaxAngle = 2.0*M_PI/3.0;    // 120 degrees
const bool GreedyProjNormalConsistency = false;

const glm::dvec3 GreedyProjColor = { 0.155, 0.186, 0.790 };
const std::string GreedyProjMaterial = "normal";
const bool GreedyProjEnabled = false;

//
// Pseudo Surface
//
const std::string PseudoSurfaceName = "Pseudo Surface";
const glm::dvec3 PseudoSurfaceColor = { 0.155, 0.186, 0.790 };
const std::string PseudoSurfaceMaterial = "normal";
const bool PseudoSurfaceEnabled = false;

//
// Clustering
//
const std::string DBSCAN_Name = "DBSCAN Labeling: ";
const bool DBSCAN_Enabled = false;
const double DBSCAN_SearchRange = 1.0;
const int DBSCAN_MinPoints = 1;