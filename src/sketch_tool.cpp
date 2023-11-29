#include "polyscope/polyscope.h"

#include "polyscope/view.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"

#include <pcl/point_cloud.h>

#include <glm/gtx/hash.hpp>

#include <stack>

#include "sketch_tool.hpp"
#include "constants.hpp"
#include "cluster.hpp"

/*
  Manage functions
*/

// Initialize screen information
void SketchTool::initSketch() {
  screenDist = polyscope::view::nearClipRatio * ScreenOffset;
  cameraOrig = polyscope::view::getCameraWorldPosition();
  cameraDir = polyscope::view::screenCoordsToWorldRay(
    glm::vec2(polyscope::view::windowWidth/2, polyscope::view::windowHeight/2)
  );
  glm::dvec3 screenOrig = cameraOrig + screenDist*cameraDir;

  // Plane on screenDist
  screen = Plane(screenOrig, cameraDir);
}

// Reset all member variables.
void SketchTool::resetSketch() {
  *currentMode = 0;

  screenDist = 0.0;
  cameraOrig = glm::dvec3(0.0, 0.0, 0.0);
  cameraDir = glm::dvec3(0.0, 0.0, 0.0);
  screen = Plane();

  sketchPoints.clear();
  basisPointsIndex.clear();
  mappedBasisConvexHull.clear();
}

/*
  Viewer functions
*/
void SketchTool::registerBasisPointsAsPointCloud(std::string name) {
  // Show basisPoints

  std::vector<std::vector<double>> points;
  std::vector<std::vector<double>> normals;
  for (int i = 0; i < basisPointsIndex.size(); i++) {
    int idx = basisPointsIndex[i];

    points.push_back({
      pointCloud->Vertices(idx, 0),
      pointCloud->Vertices(idx, 1),
      pointCloud->Vertices(idx, 2)
    });
    normals.push_back({
      pointCloud->Normals(idx, 0),
      pointCloud->Normals(idx, 1),
      pointCloud->Normals(idx, 2)
    });
  }

  polyscope::PointCloud* patchCloud = polyscope::registerPointCloud(name, points);
  patchCloud->setPointColor(BasisPointColor);
  patchCloud->setPointRadius(pointCloud->getAverageDistance()/2.0);
  patchCloud->setEnabled(false);

  polyscope::PointCloudVectorQuantity* patchVectorQuantity = patchCloud->addVectorQuantity(NormalName, normals);
  patchVectorQuantity->setVectorColor(BasisPointColor);
  patchVectorQuantity->setVectorLengthScale(pointCloud->getAverageDistance());
  patchVectorQuantity->setVectorRadius(pointCloud->getAverageDistance()/10.0);
  patchVectorQuantity->setEnabled(NormalEnabled);
  patchVectorQuantity->setMaterial(NormalMaterial);
}
void SketchTool::removePointCloud(std::string name) {
  polyscope::removePointCloud(name, false);
}

// Register/Remove curve network line with name.
// Be aware that the curve network with 
// the same name is overwritten
void SketchTool::registerSketchPointsAsCurveNetworkLine(std::string name) {
  std::vector<std::vector<double>> sketch;
  for (int i = 0; i < sketchPoints.size(); i++) {
    sketch.push_back({
      sketchPoints[i].x,
      sketchPoints[i].y,
      sketchPoints[i].z
    });
  }
  if (sketchPoints.size() == 1) {
    sketch.push_back({
      sketchPoints[0].x,
      sketchPoints[0].y,
      sketchPoints[0].z
    });
  };

  polyscope::CurveNetwork* curveNetwork = polyscope::registerCurveNetworkLine(name, sketch);
  curveNetwork->setColor(CurveNetworkColor);
  curveNetwork->setRadius(CurveNetworkRadius);  
}
void SketchTool::removeCurveNetworkLine(std::string name) {
  polyscope::removeCurveNetwork(name, false);
}

// Register/Remove curve network loop with name.
// Be aware that the curve network with 
// the same name is overwritten
void SketchTool::registerSketchPointsAsCurveNetworkLoop(std::string name) {
  std::vector<std::vector<double>> sketch;
  for (int i = 0; i < sketchPoints.size(); i++) {
    sketch.push_back({
      sketchPoints[i].x,
      sketchPoints[i].y,
      sketchPoints[i].z
    });
  }

  polyscope::CurveNetwork* curveNetwork = polyscope::registerCurveNetworkLoop(name, sketch);
  curveNetwork->setColor(CurveNetworkColor);
  curveNetwork->setRadius(CurveNetworkRadius);
}
void SketchTool::removeCurveNetworkLoop(std::string name) {
  polyscope::removeCurveNetwork(name, false);
}

/*
  Geometry functions
*/

// Cast the specified point to screen.
void SketchTool::addSketchPoint(glm::dvec3 p) {
  sketchPoints.push_back(p);
}

// Find basis points.
//  - If extendedSearch is true, extend the sketched area.
//  - Cast points of the point cloud onto the screen plane.
//  - Construct octree for the casted surface points
//  - Search for a candidate point for each discretized grid.
//    - Only points that their normals are directed to cameraOrig
//  - Detect depth with DBSCAN
void SketchTool::findBasisPoints(bool extendedSearch) {
  // If extendedSearch is true, extend the sketched area.
  if (extendedSearch) extendSketchedArea();

  // Cast points of the point cloud onto the screen plane.
  std::vector<glm::dvec3> pointsInWorldCoord;
  std::vector<glm::dvec3> normalsInWorldCoord;
  std::vector<glm::dvec3> pointsMappedOntoScreen;
  for (int i = 0; i < pointCloud->Vertices.rows(); i++) {
    glm::dvec3 p = glm::dvec3(
      pointCloud->Vertices(i, 0),
      pointCloud->Vertices(i, 1),
      pointCloud->Vertices(i, 2)
    );
    glm::dvec3 pn = glm::dvec3(
      pointCloud->Normals(i, 0),
      pointCloud->Normals(i, 1),
      pointCloud->Normals(i, 2)
    );

    // Cast a ray from p to cameraOrig onto screen.
    Ray ray(p, cameraOrig);
    Hit hitInfo = ray.castPointToPlane(&screen);
    assert(hitInfo.hit == true);

    pointsInWorldCoord.push_back(p);
    normalsInWorldCoord.push_back(pn);
    pointsMappedOntoScreen.push_back(screen.mapCoordinates(hitInfo.pos));
  }
  const int castedPointSize = pointsMappedOntoScreen.size();

  // Construct octree for the casted surface points
  pcl::PointCloud<pcl::PointXYZ>::Ptr castedCloud(new pcl::PointCloud<pcl::PointXYZ>);
  castedCloud->points.resize(castedPointSize);
  for (int i = 0; i < castedPointSize; i++) {
    castedCloud->points[i].x = pointsMappedOntoScreen[i].x;
    castedCloud->points[i].y = pointsMappedOntoScreen[i].y;
    castedCloud->points[i].z = pointsMappedOntoScreen[i].z;
  }
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(OctreeResolution);
  octree.setInputCloud(castedCloud);
  octree.addPointsFromInputCloud();

  // Search for a candidate point for each discretized grid.
  const double INF = 1e5;
  double min_x = INF, max_x = -INF;
  double min_y = INF, max_y = -INF;
  for (int i = 0; i < sketchPoints.size(); i++) {
    glm::dvec3 mappedSketchPoint = screen.mapCoordinates(sketchPoints[i]);

    min_x = std::min(min_x, mappedSketchPoint.x);
    max_x = std::max(max_x, mappedSketchPoint.x);
    min_y = std::min(min_y, mappedSketchPoint.y);
    max_y = std::max(max_y, mappedSketchPoint.y);
  }

  const double castedAverageDist = calcCastedAverageDist();
  std::vector<int>  candidatePointsIndex;
  std::vector<std::vector<double>> hasCloserNeighborPoints;
  for (double x = min_x; x < max_x; x += castedAverageDist*2.0) {
    for (double y = min_y; y < max_y; y += castedAverageDist*2.0) {
      if (!insideSketch(x, y)) continue;

      // Search for nearest neighbors with octree
      std::vector<int>    hitPointIndices;
      std::vector<float>  hitPointDistances;
      int hitPointCount = octree.radiusSearch(
        pcl::PointXYZ(x, y, 0.0),
        castedAverageDist,
        hitPointIndices,
        hitPointDistances
      );

      // Get the index of the min depth point
      double minDepth = 1e5;
      int minDepthIdx = -1;
      for (int i = 0; i < hitPointCount; i++) {
        int hitPointIdx = hitPointIndices[i];
        glm::dvec3 p = pointsInWorldCoord[hitPointIdx];
        glm::dvec3 pn = normalsInWorldCoord[hitPointIdx];

        // Discard points that their normals are directed to cameraOrig.
        if (glm::dot(pn, cameraDir) >= 0.0) continue;

        double curDepth = glm::length(p - cameraOrig);
        if (curDepth < minDepth) {
          minDepth = curDepth;
          minDepthIdx = hitPointIdx;
        }
      }

      // Update the buffer vectors
      for (int i = 0; i < hitPointCount; i++) {
        int hitPointIdx = hitPointIndices[i];
        if (hitPointIdx == minDepthIdx) {
          candidatePointsIndex.push_back(hitPointIdx);
        } else {
          glm::dvec3 pn = normalsInWorldCoord[hitPointIdx];
          if (glm::dot(pn, cameraDir) >= 0.0) continue;

          hasCloserNeighborPoints.push_back({
            pointsInWorldCoord[hitPointIdx].x,
            pointsInWorldCoord[hitPointIdx].y,
            pointsInWorldCoord[hitPointIdx].z
          });
        }
      }
    }
  }

  // Register points that has closer nearest neighbors as point cloud.
  polyscope::PointCloud* hasCloserNeighborCloud = polyscope::registerPointCloud("HasCloserNeighbor(basis)", hasCloserNeighborPoints);
  hasCloserNeighborCloud->setPointRadius(BasisPointRadius);
  hasCloserNeighborCloud->setEnabled(false);

  // Depth detection with DBSCAN
  Clustering clustering(&candidatePointsIndex, &pointsInWorldCoord, "basis");
  basisPointsIndex = clustering.executeClustering(
    DBSCAN_SearchRange*pointCloud->getAverageDistance(),
    DBSCAN_MinPoints,
    CLUSTER_MAX_SIZE
  );
}


// Find all basis points inside the sketch
//  - If extendedSearch is true, extend the sketched area.
//  - Cast points of the point cloud onto the screen plane.
//  - Judge inside/outside of the sketch.
//  - Detect depth with DBSCAN.
void SketchTool::findAllBasisPoints(bool extendedSearch) {
  if (extendedSearch) extendSketchedArea();

  // Cast points of the point cloud onto the screen plane.
  std::vector<glm::dvec3> pointsInWorldCoord;
  std::vector<glm::dvec3> pointsMappedOntoScreen;
  for (int i = 0; i < pointCloud->Vertices.rows(); i++) {
    glm::dvec3 p = glm::dvec3(
      pointCloud->Vertices(i, 0),
      pointCloud->Vertices(i, 1),
      pointCloud->Vertices(i, 2)
    );

    // Cast a ray from p to cameraOrig onto screen.
    Ray ray(p, cameraOrig);
    Hit hitInfo = ray.castPointToPlane(&screen);
    assert(hitInfo.hit == true);

    pointsInWorldCoord.push_back(p);
    pointsMappedOntoScreen.push_back(screen.mapCoordinates(hitInfo.pos));
  }
  const int castedPointSize = pointsMappedOntoScreen.size();

  // Judge inside/outside of the sketch
  std::vector<int> candidatePointsIndex;
  for (int i = 0; i < castedPointSize; i++) {
    glm::dvec3 mapped_p = pointsMappedOntoScreen[i];
    if (!insideSketch(mapped_p.x, mapped_p.y)) continue;

    candidatePointsIndex.push_back(i);
  }

  // Depth detection with DBSCAN
  Clustering clustering(&candidatePointsIndex, &pointsInWorldCoord, "basis");
  basisPointsIndex = clustering.executeClustering(
    DBSCAN_SearchRange*pointCloud->getAverageDistance(),
    DBSCAN_MinPoints,
    CLUSTER_MIN_DEPTH
  );
}

// Check whether (x, y) is inside or outside of the sketch.
//  1.  Draw a half-line parallel to the x-axis from a point.
//  2.  Determine that if there are an odd number of intersections
//      between this half-line and the polygon, it is inside, and if 
//      there are an even number, it is outside.
bool SketchTool::insideSketch(double x, double y) {
  const int sketchPointsSize = sketchPoints.size();

  int crossCount = 0;
  for (int i = 0; i < sketchPointsSize; i++) {
    glm::dvec3 mappedU = screen.mapCoordinates(sketchPoints[i]);
    glm::dvec3 mappedV = screen.mapCoordinates(sketchPoints[(i + 1) % sketchPointsSize]);

    glm::dvec2 u = glm::dvec2(mappedU.x, mappedU.y);
    glm::dvec2 v = glm::dvec2(mappedV.x, mappedV.y);

    if (crossLines(x, y, u, v)) crossCount++;
  }

  if (crossCount%2 == 0) return false;
  else return true;
}

// Check whether (x, y) is inside or outside of the mappedBasisConvexHull.
//  1.  If haven't already calculated the convex hull of the basis points,
//      then calculate it.
//  2.  Draw a half-line parallel to the x-axis from a point.
//  3.  Determine that if there are an odd number of intersections
//      between this half-line and the polygon, it is inside, and if 
//      there are an even number, it is outside.
bool SketchTool::insideBasisConvexHull(double x, double y) {
  // If the size of basisPoints is less than 3, then return false.
  if (basisPointsIndex.size() < 3) return false;

  // If haven't already calculated the convex hull of the basis points, then calculate it.
  if (mappedBasisConvexHull.size() == 0) {
    calcBasisConvexHull();
  }

  // Check whether (x, y) is inside or outside of the mappedBasisConvexHull.
  const int basisConvexHullSize = mappedBasisConvexHull.size();

  int crossCount = 0;
  for (int i = 0; i < basisConvexHullSize; i++) {
    glm::dvec2 u = mappedBasisConvexHull[i];
    glm::dvec2 v = mappedBasisConvexHull[(i + 1) % basisConvexHullSize];

    if (crossLines(x, y, u, v)) crossCount++;
  }

  if (crossCount%2 == 0) return false;
  else return true;
}

// Calculate averageDistance casted onto the screen
double SketchTool::calcCastedAverageDist() {
  double objectDist = glm::length(cameraOrig);
  double castedAverageDist = pointCloud->getAverageDistance()*screenDist/objectDist;

  return castedAverageDist;
}

// Return the pointer to member variables.
PointCloud* SketchTool::getPointCloud() {
  return pointCloud;
}
glm::dvec3 SketchTool::getCameraOrig() {
  return cameraOrig;
}
glm::dvec3 SketchTool::getCameraDir() {
  return cameraDir;
}
Plane* SketchTool::getScreen() {
  return &screen;
}
std::vector<glm::dvec3>* SketchTool::getSketchPoints() {
  return &sketchPoints;
}
std::vector<int>* SketchTool::getBasisPointsIndex() {
  return &basisPointsIndex;
}
std::vector<glm::dvec2>* SketchTool::getMappedBasisConvexHull() {
  return &mappedBasisConvexHull;
}

// Extend sketched area by averageDistance casted onto the screen.
void SketchTool::extendSketchedArea() {
  // Calculate gravity point
  glm::dvec3 gravityPoint = glm::dvec3(0.0, 0.0, 0.0);
  for (glm::dvec3 p: sketchPoints) {
    gravityPoint += p;
  }
  gravityPoint /= static_cast<double>(sketchPoints.size());

  // Extend sketched area
  double castedAverageDist = calcCastedAverageDist();
  for (int i = 0; i < sketchPoints.size(); i++) {
    glm::dvec3 p = sketchPoints[i];

    // ATTENTION: Double the average distance, because the user 
    //            should sketch with reference to the boundary of pseudo surface.
    sketchPoints[i] = p + 2.0*castedAverageDist*glm::normalize(p - gravityPoint);
  }
}

// Calculate CCW value
//  - ccw > 0.0: left turn
//  - ccw < 0.0: right turn
//  - ccw == 0.0: parallel
double SketchTool::calcCCW(glm::dvec2 p, glm::dvec2 a, glm::dvec2 b) {
  glm::dvec2 u = a - p;
  glm::dvec2 v = b - p;
  return u.x*v.y - v.x*u.y;
}

// Calculate the convex hull of basisPoints O(n\log{n})
//  1. Find point P with the lowest y-coordinate.
//  2. Sort the points in increasing order of the angle
//     they and the point P make with the x-axis.
//     (Break ties by increasing distance.)
//  3. Traversing all points considering "left turn" and "right turn"
//
// implemented referencing https://en.wikipedia.org/wiki/Graham_scan
void SketchTool::calcBasisConvexHull() {
  const int basisPointsSize = basisPointsIndex.size();

  // Initialize vector of mapped basis points.
  std::vector<glm::dvec2> mappedBasisPoints(basisPointsSize);
  for (int i = 0; i < basisPointsSize; i++) {
    int idx = basisPointsIndex[i];
    glm::dvec3 p = glm::dvec3(
      pointCloud->Vertices(idx, 0),
      pointCloud->Vertices(idx, 1),
      pointCloud->Vertices(idx, 2)
    );

    // Cast a ray from p to cameraOrig onto screen.
    Ray ray(p, cameraOrig);
    Hit hitInfo = ray.castPointToPlane(&screen);
    assert(hitInfo.hit == true);
    glm::dvec3 mappedP = screen.mapCoordinates(hitInfo.pos);
    mappedBasisPoints[i] = glm::dvec2(mappedP.x, mappedP.y);
  }

  //  1. Find point P with the lowest y-coordinate.
  double min_y = mappedBasisPoints[0].y, minIndex = 0;
  for (int i = 1; i < basisPointsSize; i++) {
    glm::dvec2 p = mappedBasisPoints[i];
    if (p.y < min_y || (p.y == min_y && p.x < mappedBasisPoints[minIndex].x)) {
      min_y = p.y;
      minIndex = i;
    }
  }

  //  2. Sort the points in increasing order of the angle
  //     they and the point P make with the x-axis.
  //     (Break ties by increasing distance.)
  std::swap(mappedBasisPoints[0], mappedBasisPoints[minIndex]);
  glm::dvec2 P = mappedBasisPoints[0];
  std::sort(mappedBasisPoints.begin() + 1, mappedBasisPoints.end(), [&](const glm::dvec2 &l, const glm::dvec2 &r) {
    double ccw = calcCCW(P, l, r);
    if (ccw > 0.0) return true;
    else if (ccw < 0.0) return false;
    else {
      if (glm::length(l - P) < glm::length(r - P)) return true;
      else return false;
    }
  });

  //  3. Traversing all points considering "left turn" and "right turn"
  std::stack<glm::dvec2> pointStack;
  pointStack.push(mappedBasisPoints[0]);
  pointStack.push(mappedBasisPoints[1]);

  for (int i = 2; i < basisPointsSize; i++) {
    // p -> a -> b
    glm::dvec2 a = pointStack.top(); 
    pointStack.pop();

    glm::dvec2 b = mappedBasisPoints[i];

    while (!pointStack.empty() && calcCCW(pointStack.top(), a, b) <= 0.0) {
      a = pointStack.top();
      pointStack.pop();
    }
    pointStack.push(a);
    pointStack.push(b);
  }

  // Extract points from stack
  while (!pointStack.empty()) {
    mappedBasisConvexHull.push_back(pointStack.top());
    pointStack.pop();
  }
  std::reverse(mappedBasisConvexHull.begin(), mappedBasisConvexHull.end());
}

// Return whether half-line from (x, y) crosses u-v.
bool SketchTool::crossLines(double x, double y, glm::dvec2 u, glm::dvec2 v) {
  const double EPS = 1e-5; 

  // If u, v are too close to (x, y), then offset. 
  if (std::abs(u.y - y) < EPS) {
    if (u.y - y >= 0.0d)  u.y += EPS;
    else u.y -= EPS;
  }
  if (std::abs(v.y - y) < EPS) {
    if (v.y - y >= 0.0d)  v.y += EPS;
    else v.y -= EPS;
  }

  // If (u, v) is parallel to x-axis, then skip it.
  if (std::abs(u.y - y) < EPS && std::abs(v.y - y) < EPS) return false;

  // If u, v are in the same side of the half-line, then skip it.
  if ((u.y - y)*(v.y - y) >= 0.0d) return false;

  // If (u, v) doesn't intersect the half-line, then skip it.
  double crossX = u.x + (v.x - u.x)*std::abs(y - u.y)/std::abs(v.y - u.y);
  if (crossX < x) return false;
  else return true;
}