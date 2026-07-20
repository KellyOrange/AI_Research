#include <cstdio>
#include <vector>
#include <chrono>

#include "Vec2.h"
#include "Portal.h"
#include "pathsmooth.h"
#include "Metrics.h"
#include "JsonExport.h"

CorridorResult RunCorridor(const char* corridorName, const Vec2& start, const Vec2& goal, const std::vector<Portal>& portals) {
    using Clock = std::chrono::high_resolution_clock;

    printf("=========================================\n");
    printf("Corridor: %s\n", corridorName);
    printf("=========================================\n");

    auto t0 = Clock::now();
    std::vector<Vec2> funnelPath = Funnel(start, goal, portals);
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
    result.funnelPath = funnelPath;
    result.rubberPath = rubberPath;
    result.splinePath = splinePath;
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

    ExportToJSON("paths.json", allResults);
    printf("\nExported path data to paths.json -- open index.html to view it.\n");

    return 0;
}