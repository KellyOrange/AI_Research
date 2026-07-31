#include <cstdio>
#include <vector>
#include <chrono>

#include "vec2.h"
#include "portal.h"
#include "pathsmooth.h"
#include "metrics.h"
#include "jsonExport.h"
#include "navMesh.h"

CorridorResult RunCorridor(const char* corridorName, const Vec2& start, const Vec2& goal,
                            const std::vector<Portal>& portals, const std::vector<Rect>& obstacles = {}) {
    using Clock = std::chrono::high_resolution_clock;

    printf("=========================================\n");
    printf("Corridor: %s\n", corridorName);
    printf("=========================================\n");

    auto t0 = Clock::now();
    std::vector<FunnelStep> funnelTrace;
    std::vector<Vec2> funnelPath = FunnelWithTrace(start, goal, portals, funnelTrace);
    auto t1 = Clock::now();

    std::vector<Vec2> rubberPath = RubberBand(start, goal, portals);
    auto t2 = Clock::now();

    std::vector<Vec2> rawPath = RawPathFromPortals(start, goal, portals);
    std::vector<Vec2> splinePath = CatmullRom(rawPath);
    auto t3 = Clock::now();

    PrintPath("Funnel (string-pulling)", funnelPath);
    PrintPath("Rubber-banding", rubberPath);
    PrintPath("Catmull-Rom spline", splinePath);

    printf("Timing (microseconds):\n");
    printf("  Funnel:      %.2f us\n", std::chrono::duration<double, std::micro>(t1 - t0).count());
    printf("  RubberBand:  %.2f us\n", std::chrono::duration<double, std::micro>(t2 - t1).count());
    printf("  CatmullRom:  %.2f us\n\n", std::chrono::duration<double, std::micro>(t3 - t2).count());

    CorridorResult result;
    result.name = corridorName;
    result.start = start;
    result.goal = goal;
    result.portals = portals;
    result.rawPath = rawPath;
    result.funnelPath = funnelPath;
    result.rubberPath = rubberPath;
    result.splinePath = splinePath;
    result.obstacles = obstacles;
    result.funnelTrace = funnelTrace;
    result.funnelTimeUs = std::chrono::duration<double, std::micro>(t1 - t0).count();
    result.rubberTimeUs = std::chrono::duration<double, std::micro>(t2 - t1).count();
    result.splineTimeUs = std::chrono::duration<double, std::micro>(t3 - t2).count();
    return result;
}

int main() {
    std::vector<CorridorResult> allResults;

    // ---------------------------------------------------------------
    // Corridor 1: gentle bent hallway (original test case).
    // ---------------------------------------------------------------
    {
        Vec2 start = { 0.0f, 0.0f };
        Vec2 goal  = { 8.0f, 6.0f };
        std::vector<Portal> portals = {
            { {1.0f, 2.0f}, {1.0f, -1.0f} },
            { {3.0f, 3.0f}, {3.0f, 0.0f} },
            { {5.0f, 5.0f}, {5.0f, 1.0f} },
            { {7.0f, 7.0f}, {7.0f, 2.0f} },
        };
        allResults.push_back(RunCorridor("Gentle bend", start, goal, portals));
    }

    // ---------------------------------------------------------------
    // Corridor 2: sharp narrow zigzag (S-shaped, very tight portals).
    // This corridor pinches down to near-zero width partway through,
    // which is where Catmull-Rom's lack of wall-awareness is most
    // likely to show visible overshoot once you add rendering.
    // ---------------------------------------------------------------
    {
        Vec2 start = { 0.0f, 0.0f };
        Vec2 goal  = { 6.0f, 0.0f };
        std::vector<Portal> portals = {
            { {1.0f, 1.0f}, {1.0f, 0.5f} },
            { {2.0f, 1.0f}, {2.0f, 0.5f} },
            { {3.0f, -1.0f}, {3.0f, -0.5f} },
            { {4.0f, -1.0f}, {4.0f, -0.5f} },
            { {5.0f, 0.5f}, {5.0f, 0.0f} },
        };
        allResults.push_back(RunCorridor("Sharp zigzag (S-curve)", start, goal, portals));

        printf("Portal bounds for corridor 2 (for checking spline overshoot):\n");
        for (size_t i = 0; i < portals.size(); ++i) {
            printf("  portal %zu: left=(%.2f,%.2f) right=(%.2f,%.2f)\n",
                i, portals[i].left.x, portals[i].left.y, portals[i].right.x, portals[i].right.y);
        }
    }

    // ---------------------------------------------------------------
    // Corridor 3: REAL generated level. A grid nav mesh is built from
    // obstacle rectangles, A* finds a triangle path across it, and the
    // real portal sequence crossed is extracted -- fed into the exact
    // same Funnel/RubberBand/CatmullRom functions as the hand-built
    // corridors above. This is the actual demo pipeline end-to-end.
    // ---------------------------------------------------------------
    {
        float levelWidth = 20.0f, levelHeight = 14.0f, cellSize = 2.0f;

        // Narrow corridor near the start, a big OPEN ROOM in the middle
        // (no obstacles at all there), then another narrow corridor near
        // the goal. Walls are sized/positioned to align cleanly with the
        // 2-unit grid tiles, so tiles (and portals) stay wide even right
        // next to the walls. A 1-unit tile size caps every portal at 1
        // unit wide EVERYWHERE, even in open space -- which is why the
        // funnel algorithm had no real room to cut corners before, even
        // once the portal left/right bug was fixed.
        std::vector<Rect> obstacles = {
            { 4.0f, 0.0f, 6.0f, 10.0f },   // narrow corridor wall near start
            { 16.0f, 4.0f, 18.0f, 14.0f }, // narrow corridor wall near goal
        };

        NavMesh mesh = BuildGridNavMesh(levelWidth, levelHeight, cellSize, obstacles);

        Vec2 start = { 1.0f, 1.0f };
        Vec2 goal  = { 18.0f, 12.0f };

        int startTri = FindTriangleContaining(mesh, start);
        int goalTri = FindTriangleContaining(mesh, goal);

        if (startTri == -1 || goalTri == -1) {
            printf("Corridor 3: start or goal point is not on the nav mesh (inside an obstacle?)\n");
        } else {
            std::vector<int> triPath = FindTrianglePath(mesh, startTri, goalTri);
            if (triPath.empty()) {
                printf("Corridor 3: A* found no path between start and goal.\n");
            } else {
                std::vector<Portal> portals = ExtractPortals(mesh, triPath, start);
                printf("Corridor 3: nav mesh has %zu triangles, A* path crosses %zu triangles / %zu portals\n\n",
                    mesh.triangles.size(), triPath.size(), portals.size());
                allResults.push_back(RunCorridor("Generated level (real nav mesh + A*)", start, goal, portals, obstacles));
            }
        }
    }

    ExportToJSON("paths.json", allResults);
    printf("\nExported path data to paths.json -- open index.html to view it.\n");

    return 0;
}