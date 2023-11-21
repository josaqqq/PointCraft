#pragma once

//
// Scene parameters
//
const std::array<float, 4> BackgroundColor = { 0.025, 0.025, 0.025, 0.000 };
const int WindowWidth = 1024;
const int WindowHeight = 1024;

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

//
// Octree parameters
//
const double OctreeResolution = 64.0;

//
// Patch parameters
//
const int PatchSize = 50;
const double depthInterval = 5.0; // Multiple with averageDistance of input point cloud.

const glm::dvec3 DiscretizedPointColor = { 1.000, 0.000, 0.000 };
const double DiscretizedPointRadius = 0.0025;

const glm::dvec3 BasisPointColor = { 0.000, 1.000, 0.000 };
const double BasisPointRadius = 0.0025;

const std::string TracePrefix = "Trace: ";
const std::string CastPrefix = "Cast: ";
const std::string SketchPrefix = "Sketch: ";

//
//  Curve Network
//
const glm::dvec3 CurveNetworkColor = { 0.000, 1.000, 0.000 };
const double CurveNetworkRadius   = 0.00025;

//
// Poisson Surface Reconstruction
//
const std::string PoissonName = "Poisson Surface Reconstruction";
const int PoissonMaxDepth = 5;
const glm::dvec3 PoissonColor = { 0.155, 0.186, 0.790 };
const std::string PoissonMaterial = "normal";

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
const bool MLSPolynomialFitFlag = true;
const double MLSSearchRadius = 0.3;

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

//
// Clustering
//
const double DBSCAN_SearchRange = 1.0;
const int DBSCAN_MinPoints = 1;