#pragma once

// "
//  (1) The user marks a support region in the point cloud, where the 
//      reconstructed surface serves as a proxy for the 3D selection tool;
//  (2) we fit a plane to this region using PCA and project the points of
//      the support region onto this plane;
//  (3) this produces a sampling of a height field h overthe plane, 
//      which we reconstruct using a thin plate spline;
//  (4) the user can interactively sample points from this height field to fill
//      the hole, until satisfied with the result.
// "
// Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold,
// and Daniele Panozzo. 2017. Field-aligned online surface reconstruction. 
// ACM Trans. Graph. 36, 4, Article 77 (August 2017), 13 pages. https://doi.org/10.1145/3072959.3073635


