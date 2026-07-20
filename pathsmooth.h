#pragma once

#include <vector>
#include "vec2.h"
#include "portal.h"

// Classic Simple Stupid Funnel Algorithm (Mikko Mononen's formulation).
// Input: start point, goal point, and the portals crossed in order.
// Output: the smoothed (taut) path through the corridor.
std::vector<Vec2> Funnel(const Vec2& start, const Vec2& goal, const std::vector<Portal>& portals);

// Builds a "raw" waypoint path from the portal sequence, the way a
// grid-based pathfinder would hand you a path (start -> each portal's
// midpoint -> goal). This is the common input the other two methods
// smooth, since rubber-banding/Catmull-Rom aren't funnel-specific.
std::vector<Vec2> RawPathFromPortals(const Vec2& start, const Vec2& goal, const std::vector<Portal>& portals);

// Rubber-banding: pull interior points toward the average of their
// neighbors over several iterations, like a taut rubber band relaxing.
// Each point is clamped back onto its portal segment afterward so the
// path can't relax itself straight through a wall.
std::vector<Vec2> RubberBand(const Vec2& start, const Vec2& goal, const std::vector<Portal>& portals,
                              int iterations = 20, float pullFactor = 0.5f);

// Catmull-Rom spline: fits a smooth curve through the raw waypoints.
// Note this does NOT know about corridor walls at all -- it can cut
// through geometry the funnel/rubber-band methods would avoid.
std::vector<Vec2> CatmullRom(const std::vector<Vec2>& raw, int samplesPerSegment = 8);