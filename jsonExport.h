#pragma once

#include <vector>
#include <string>
#include "Vec2.h"
#include "Portal.h"

// One corridor's worth of data: the portals plus each method's smoothed path.
struct CorridorResult {
    std::string name;
    Vec2 start;
    Vec2 goal;
    std::vector<Portal> portals;
    std::vector<Vec2> funnelPath;
    std::vector<Vec2> rubberPath;
    std::vector<Vec2> splinePath;
};

// Writes a list of corridor results to a JSON file that index.html can load.
void ExportToJSON(const std::string& filepath, const std::vector<CorridorResult>& corridors); 