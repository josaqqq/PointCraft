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
const double PointCloudBoundingBoxSide = 1.0d;
const double PointCloudDownsampleVoxel = 0.025d;

const std::string PointName = "Point Cloud";
const glm::dvec3 PointColor = { 1.000, 1.000, 1.000 };
const double PointRadius = 0.002;

const std::string NormalName = "normal vector";
const glm::dvec3 NormalColor =  { 1.000, 1.000, 1.000 };
const double NormalLength = 0.015;
const double NormalRadius = 0.001;
const bool NormalEnabled = true;
const std::string NormalMaterial = "flat";

const std::string SurfacePointName = "Surface Points";
const int SurfacePointNum = 10;

//
// Surface points
//
const glm::dvec3 SurfacePointColor = { 0.000, 1.000, 0.000 };
const double SurfacePointRadius = 0.0025;
const std::string SurfaceMaterial = "flat";

//
// Octree parameters
//
const double OctreeResolution = 0.025;  // The length of the smallest voxels at lowest octree level.

//
// Patch parameters
//
const std::string SketchPointName = "Sketch Points";
const glm::dvec3 SketchPointColor = { 0.000, 0.000, 1.000 };

const glm::dvec3 BasisPointColor = { 0.000, 1.000, 0.000 };
const double BasisPointRadius = 0.0025;
const double BasisNormalLength = 0.05;
const double BasisNormalRadius = 0.003;

const std::string SketchPrefix = "Sketch: ";

//
//  Curve Network
//
const glm::dvec3 CurveNetworkColor = { 0.000, 1.000, 0.000 };
const double CurveNetworkRadius   = 0.00020;
const std::string CurveNetworkMaterial = "flat";

//
// Poisson Surface Reconstruction
//
const int PoissonMaxDepth = 5;
const int PoissonThreadNum = 4;

const std::string PoissonName = "Poisson Surface Reconstruction";
const glm::dvec3 PoissonColor = { 0.000, 0.196, 1.000 };
const glm::dvec3 PoissonBackgroundColor = { 1.000, 0.686, 0.000 };
const std::string PoissonMaterial = "clay";
const polyscope::BackFacePolicy PoissonBackFacePolicy = polyscope::BackFacePolicy::Custom;
const bool PoissonEnabled = false;

//
// Moving Least Squares
//
const int MLS_SprayNearestNeighbors = 30;
const int MLS_SpraySize = 4;

const int MLSPolynomialOrder = 2;

const std::string MLSName = "Interpolation MLS";
const glm::dvec3 MLSColor = { 0.000, 0.000, 1.000 };

//
// Pseudo Surface
//
const std::string PseudoSurfaceName = "Pseudo Surface";
const glm::dvec3 PseudoSurfaceColor =  { 0.000, 0.196, 1.000 };
const glm::dvec3 PseudoSurfaceBackgroundColor = { 1.000, 0.686, 0.000 };
const std::string PseudoSurfaceMaterial = "clay";
const polyscope::BackFacePolicy PseudoSurfaceBackFacePolicy = polyscope::BackFacePolicy::Custom;
const bool PseudoSurfaceEnabled = false;

//
// Clustering
//
const double DBSCAN_SearchRange = 1.0;
const int DBSCAN_MinPoints = 1;

const std::string DBSCAN_Name = "DBSCAN Labeling: ";
const bool DBSCAN_Enabled = false;