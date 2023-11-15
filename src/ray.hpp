#pragma once

#include "polyscope/polyscope.h"
#include "polyscope/view.h"

class Hit {
  public:
    Hit(glm::vec2 screenCoord) : screenCoord(screenCoord) {}

    bool hit;         // hit or not
    double t;         // ray parameter
    double depth;     // distance from "camera plane" to the point
    
    glm::vec3 pos;    // hit point
    glm::vec3 normal; // hit point normal

    glm::vec2 screenCoord;  // Coordinates of the origin of this ray
};

class Ray {
  public:
    Ray(double xScreen, double yScreen);

    // Calculate the depth from the point 
    // to the plane that is orthogonal to the 
    // frontVec of the camera.
    double calcDepthFromCamera(glm::vec3 point);

    // Check whether this ray intersect
    // with the sphere specified with 
    // center and radius.
    Hit checkSphere(glm::vec3 center, double radius);

    // Search for the nearest neighbor point
    // along the specified line.
    // The search range is within the range
    // of the searchRadius.
    Hit searchNeighborPoints(double searchRadius);

    // Cast the point to the specified plane
    // that is parallel to "camera plane".
    Hit castPointToPlane(double depth);

  private:
    glm::vec2 screenCoord;  // Coordinates on the screen
    glm::vec3 orig;         // Coordinates of the origin of this ray
    glm::vec3 rayDir;       // Vector of the direction of this ray
    glm::vec3 cameraDir;    // Vector of the direction of the camera
};
