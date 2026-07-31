#include "navMesh.h"
#include <map>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
#include "vec2.h" // for Cross()

// A grid-space vertex, used only while building adjacency (exact
// integer coordinates avoid any floating point matching issues).
using GridVertex = std::pair<int, int>;

static float DistanceBetween(const Vec2& a, const Vec2& b) {
    float dx = b.x - a.x, dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

NavMesh BuildGridNavMesh(float width, float height, float cellSize, const std::vector<Rect>& obstacles) {
    NavMesh mesh;
    int cols = (int)(width / cellSize);
    int rows = (int)(height / cellSize);

    // edgeMap: canonical (min vertex, max vertex) -> list of (triangleIndex, localEdgeIndex)
    // seen for that edge so far. Once an edge has been seen twice, the
    // two triangles that share it are neighbors across that edge.
    std::map<std::pair<GridVertex, GridVertex>, std::vector<std::pair<int, int>>> edgeMap;

    auto addTriangle = [&](Vec2 v0, Vec2 v1, Vec2 v2, GridVertex g0, GridVertex g1, GridVertex g2) {
        int idx = (int)mesh.triangles.size();
        mesh.triangles.push_back({ { v0, v1, v2 } });
        mesh.neighbor.push_back({ -1, -1, -1 });

        GridVertex g[3] = { g0, g1, g2 };
        for (int e = 0; e < 3; ++e) {
            GridVertex a = g[e];
            GridVertex b = g[(e + 1) % 3];
            auto key = (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
            edgeMap[key].push_back({ idx, e });
        }
    };

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            Rect cellRect = { col * cellSize, row * cellSize, (col + 1) * cellSize, (row + 1) * cellSize };
            bool blocked = false;
            for (const auto& obs : obstacles) {
                if (cellRect.Overlaps(obs)) { blocked = true; break; }
            }
            if (blocked) continue;

            Vec2 BL = { col * cellSize, row * cellSize };
            Vec2 BR = { (col + 1) * cellSize, row * cellSize };
            Vec2 TL = { col * cellSize, (row + 1) * cellSize };
            Vec2 TR = { (col + 1) * cellSize, (row + 1) * cellSize };

            GridVertex gBL = { col, row };
            GridVertex gBR = { col + 1, row };
            GridVertex gTL = { col, row + 1 };
            GridVertex gTR = { col + 1, row + 1 };

            // Split along the BL-TR diagonal, both halves CCW.
            addTriangle(BL, BR, TR, gBL, gBR, gTR);
            addTriangle(BL, TR, TL, gBL, gTR, gTL);
        }
    }

    // Any edge that was recorded exactly twice is shared by two
    // triangles -- wire them up as neighbors across that edge.
    for (auto& entry : edgeMap) {
        auto& occurrences = entry.second;
        if (occurrences.size() == 2) {
            auto [triA, edgeA] = occurrences[0];
            auto [triB, edgeB] = occurrences[1];
            mesh.neighbor[triA][edgeA] = triB;
            mesh.neighbor[triB][edgeB] = triA;
        }
    }

    return mesh;
}

int FindTriangleContaining(const NavMesh& mesh, const Vec2& p) {
    for (size_t i = 0; i < mesh.triangles.size(); ++i) {
        const Triangle& t = mesh.triangles[i];
        // CCW triangle: point is inside if it's left of (or on) all 3 edges.
        if (Cross(t.v[0], t.v[1], p) >= -1e-4f &&
            Cross(t.v[1], t.v[2], p) >= -1e-4f &&
            Cross(t.v[2], t.v[0], p) >= -1e-4f) {
            return (int)i;
        }
    }
    return -1;
}

std::vector<int> FindTrianglePath(const NavMesh& mesh, int startTri, int goalTri) {
    if (startTri < 0 || goalTri < 0) return {};
    size_t n = mesh.triangles.size();

    std::vector<float> gScore(n, std::numeric_limits<float>::infinity());
    std::vector<int> cameFrom(n, -1);
    std::vector<bool> visited(n, false);

    auto heuristic = [&](int t) {
        return DistanceBetween(mesh.triangles[t].Centroid(), mesh.triangles[goalTri].Centroid());
    };

    using QueueItem = std::pair<float, int>; // (fScore, triangleIndex)
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;

    gScore[startTri] = 0.0f;
    open.push({ heuristic(startTri), startTri });

    while (!open.empty()) {
        int current = open.top().second;
        open.pop();
        if (visited[current]) continue;
        visited[current] = true;

        if (current == goalTri) {
            std::vector<int> path;
            for (int t = goalTri; t != -1; t = cameFrom[t]) path.push_back(t);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int neighborTri : mesh.neighbor[current]) {
            if (neighborTri < 0 || visited[neighborTri]) continue;
            float cost = DistanceBetween(mesh.triangles[current].Centroid(), mesh.triangles[neighborTri].Centroid());
            float tentativeG = gScore[current] + cost;
            if (tentativeG < gScore[neighborTri]) {
                gScore[neighborTri] = tentativeG;
                cameFrom[neighborTri] = current;
                open.push({ tentativeG + heuristic(neighborTri), neighborTri });
            }
        }
    }
    return {}; // no path found
}

std::vector<Portal> ExtractPortals(const NavMesh& mesh, const std::vector<int>& triPath, const Vec2& start) {
    std::vector<Portal> portals;
    (void)start; // no longer needed -- left/right is now derived from mesh winding alone.

    for (size_t i = 0; i + 1 < triPath.size(); ++i) {
        int t = triPath[i];
        int nextT = triPath[i + 1];
        const Triangle& tri = mesh.triangles[t];

        // Find which edge of triangle t borders triangle nextT.
        int edgeIndex = -1;
        for (int e = 0; e < 3; ++e) {
            if (mesh.neighbor[t][e] == nextT) { edgeIndex = e; break; }
        }
        if (edgeIndex == -1) continue; // shouldn't happen if triPath is valid

        // Every triangle is stored counter-clockwise, so walking this
        // edge in the order it's stored in triangle t (a -> b) always
        // keeps t's interior on the left. That means, from the point of
        // view of someone walking forward out of t and into nextT, b is
        // always the LEFT side of the doorway and a is always the RIGHT
        // side -- a fixed rule from mesh topology alone, independent of
        // path direction. (The previous direction-based heuristic here
        // was unreliable and produced a corrupted, oscillating path.)
        Vec2 a = tri.v[edgeIndex];
        Vec2 b = tri.v[(edgeIndex + 1) % 3];

        Portal portal;
        portal.left = b;
        portal.right = a;
        portals.push_back(portal);
    }

    return portals;
}