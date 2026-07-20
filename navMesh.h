#pragma once

#include <vector>
#include <array>
#include "vec2.h"
#include "portal.h"
#include "rect.h"

struct Triangle {
    Vec2 v[3]; // always stored counter-clockwise (CCW)

    Vec2 Centroid() const {
        return { (v[0].x + v[1].x + v[2].x) / 3.0f, (v[0].y + v[1].y + v[2].y) / 3.0f };
    }
};

struct NavMesh {
    std::vector<Triangle> triangles;
    // neighbor[t][e] = index of the triangle across edge e of triangle t
    // (edge e connects v[e] to v[(e+1)%3]), or -1 if that edge is a wall.
    std::vector<std::array<int, 3>> neighbor;
};

// Builds a nav mesh by rasterizing a level into a uniform grid, marking
// a cell blocked if it overlaps any obstacle, and splitting each free
// cell into two CCW triangles along a consistent diagonal. Adjacency
// between all triangles (including across cell boundaries) is computed
// automatically by matching shared edges.
NavMesh BuildGridNavMesh(float width, float height, float cellSize, const std::vector<Rect>& obstacles);

// Finds the index of the triangle containing point p (via point-in-
// triangle test), or -1 if none contains it (e.g. point is inside an
// obstacle or out of the level bounds).
int FindTriangleContaining(const NavMesh& mesh, const Vec2& p);

// A* search over the triangle adjacency graph, using centroid-to-
// centroid distance as both edge cost and heuristic. Returns the
// sequence of triangle indices from startTri to goalTri (inclusive),
// or an empty vector if no path exists.
std::vector<int> FindTrianglePath(const NavMesh& mesh, int startTri, int goalTri);

// Converts a triangle-index path into the portal sequence (the shared
// edges crossed along the way) that Funnel/RubberBand/CatmullRom
// expect. Left/right for each portal is resolved based on direction
// of travel so the funnel algorithm gets a consistent corridor.
std::vector<Portal> ExtractPortals(const NavMesh& mesh, const std::vector<int>& triPath, const Vec2& start);