#pragma once

#include <vector>
#include <string>
#include "vec2.h"
#include "portal.h"
#include "rect.h"

// One corridor's worth of data: the portals plus each method's smoothed path.
struct CorridorResult {
    std::string name;
    Vec2 start;
    Vec2 goal;
    std::vector<Portal> portals;
    std::vector<Vec2> rawPath;
    std::vector<Vec2> funnelPath;
    std::vector<Vec2> rubberPath;
    std::vector<Vec2> splinePath;
    std::vector<Rect> obstacles;

    // Timing in microseconds for each smoothing method (measured in main.cpp).
    double funnelTimeUs = 0.0;
    double rubberTimeUs = 0.0;
    double splineTimeUs = 0.0;
};

// Writes a list of corridor results to a JSON file that index.html can load.
void ExportToJSON(const std::string& filepath, const std::vector<CorridorResult>& corridors);