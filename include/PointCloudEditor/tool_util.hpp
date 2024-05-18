#pragma once

#include "point_cloud.hpp"
#include "sketch.hpp"
#include "camera.hpp"

enum Mode {
	MODE_NONE,
	MODE_HOLE_FILL,
  MODE_ERASER,
  MODE_SPRAY,
};

class ToolUtil {
	public:
		ToolUtil(int* currentMode, PointCloud* pointCloud, Sketch* sketch, Camera* camera)
		: currentMode(currentMode), pointCloud(pointCloud), sketch(sketch), camera(camera) {}
		virtual ~ToolUtil() {}

		/* Tool Management */

		virtual void runTool() {};
    void initTool();
		void resetTool();

    // Reset member variables (vector and set)
    void resetSurfacePointsIndex();
    void resetBasisPoints();
    void resetSketchPoints();
    void resetBasisConvexHull();    

		/* Geometry Processing */

    // Compute the surface points where mouse is currently hovering
    // and then update surfacePointsIndex
    //  - p: the screen coordinate [-1.0f, 1.0f]
    //  - K_size: the selected nearest neighbors size
    void updateSurfacePoints(SketchPoint p, int K_size);

		// Add the specified point to sketchPoints
		void addSketchPoint(SketchPoint p);

		// Find basis points
		//	1. Cast rays from camera position to grid
		//	2. Process ray-marching to find a candidate point for each ray
		void findBasisPoints(
      bool addSurfacePoints,
      int clusteringMode
    );

    // Check whether (x, y) is inside or outside of the sketch.
    //  1.  Draw a half-line parallel to the x-axis from a point.
    //  2.  Determine that if there are an odd number of intersections
    //      between this half-line and the polygon, it is inside, and if 
    //      there are an even number, it is outside.
    bool insideSketch(SketchPoint p);

    // Check whether (x, y) is inside or outside of the basisConvexHull.
    //  1.  If haven't already calculated the convex hull of the basis points,
    //      then calculate it.
    //  2.  Draw a half-line parallel to the x-axis from a point.
    //  3.  Determine that if there are an odd number of intersections
    //      between this half-line and the polygon, it is inside, and if 
    //      there are an even number, it is outside.
    bool insideBasisConvexHull(SketchPoint p);		

    // Filter the interpolated points in depth
    //	1. Construct PointCloud class on targetVertices
    //	2. Cast a ray for each discretized grid and select candidate points
    //	3. Detect depth with DBSCAN
    //  
    std::vector<Vertex> filterWithDepth(std::vector<Vertex> &targetVertices);

    // Filter the interpolated points in voxel, whose side is equal to averageDistance
    //	1. Compute whether each point is a candidate point
    //	2. Search the point cloud and target points for nearest neighbors for each points
    //	3. Select candidate points for each voxel.
    std::vector<Vertex> filterWithVoxel(std::vector<Vertex> &targetVertices);

    // Calculate averageDistance casted onto the screen
    double calcCastedAverageDist();

		/* Get Member Variables */
    PointCloud* getPointCloud();
    Sketch*     getSketch();
    Camera*     getCamera();

    std::set<int>*            getSurfacePointsIndex();
    std::vector<Vertex>*      getBasisPoints();
    std::vector<SketchPoint>* getSketchPoints();
    std::vector<SketchPoint>* getBasisConvexHull();

    PointCloud* getBasisPointCloud();

  protected:
    // Calculate CCW value
    //  - ccw > 0.0: left turn
    //  - ccw < 0.0: right turn
    //  - ccw = 0.0: parallel
    double calcCCW(SketchPoint p, SketchPoint a, SketchPoint b);

    // Calculate the convex hull of basisPoints O(n\log{n})
    //  1. Find point P with the lowest y-coordinate.
    //  2. Sort the points in increasing order of the angle
    //     they and the point P make with the x-axis.
    //     (Break ties by increasing distance.)
    //  3. Traversing all points considering "left turn" and "right turn"
    //
    // implemented referencing https://en.wikipedia.org/wiki/Graham_scan
    void computeBasisConvexHull();

    // Return whether half-line from (x, y) coresses u-v
    bool crossLines(SketchPoint p, SketchPoint u, SketchPoint v);

    int* currentMode;       // current selected mode
    
    PointCloud* pointCloud; // registered point cloud
    Sketch* sketch;         // class for sketch rendering
    Camera* camera;         // regsitered camera

    std::set<int>            surfacePointsIndex;  // The indices of surface points where the mouse is hovering
    std::vector<Vertex>      basisPoints;         // Selected basis points
    std::vector<SketchPoint> sketchPoints;        // Sketched points on the camera screen
    std::vector<SketchPoint> basisConvexHull;     // Convex hull of the basis points mapped onto screen

    PointCloud basisPointCloud;
};
