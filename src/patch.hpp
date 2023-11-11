#pragma once

#include "modes.hpp"

// TODO: ImGuiIo is not necessary for parameter.

// Pick a point by the mouse position, 
// then add the point to Patch.
void tracePoints(ImGuiIO &io, Mode &currentMode);

// Cast a ray from the mouse position,
// then add the intersection with sphere
// to Patch.
void castPointToSphere(ImGuiIO &io, Mode &currentMode, glm::vec3 center, double radius);