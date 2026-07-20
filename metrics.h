#pragma once

#include <vector>
#include "Vec2.h"

// Total length of a path (sum of segment lengths).
float PathLength(const std::vector<Vec2>& path);

// Counts direction changes greater than angleThresholdDeg, so
// dead-straight runs don't get flagged as turns.
int TurnCount(const std::vector<Vec2>& path, float angleThresholdDeg = 5.0f);

// Prints a path's points plus its length/turn-count metrics.
void PrintPath(const char* label, const std::vector<Vec2>& path);