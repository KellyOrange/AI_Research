#include "pathsmooth.h"

std::vector<Vec2> Funnel(const Vec2& start, const Vec2& goal, const std::vector<Portal>& portals) {
    std::vector<FunnelStep> discardedTrace;
    return FunnelWithTrace(start, goal, portals, discardedTrace);
}

std::vector<Vec2> FunnelWithTrace(const Vec2& start, const Vec2& goal, const std::vector<Portal>& portals,
                                   std::vector<FunnelStep>& outTrace) {
    outTrace.clear();

    std::vector<Vec2> path;
    path.push_back(start);

    Vec2 apex = start;
    Vec2 left = start;
    Vec2 right = start;
    size_t apexIndex = 0, leftIndex = 0, rightIndex = 0;

    // Append the goal as a zero-width final portal so the loop handles it uniformly.
    std::vector<Portal> pts = portals;
    pts.push_back({ goal, goal });

    for (size_t i = 0; i < pts.size(); ++i) {
        const Vec2& newLeft = pts[i].left;
        const Vec2& newRight = pts[i].right;
        bool committed = false;
        Vec2 committedVertex = {};

        // --- update right side of the funnel ---
        if (Cross(apex, right, newRight) <= 0.0f) {
            if ((apex.x == right.x && apex.y == right.y) || Cross(apex, left, newRight) > 0.0f) {
                right = newRight;
                rightIndex = i;
            }
            else {
                // Right crosses over left: apex moves to left, restart from there.
                path.push_back(left);
                committed = true;
                committedVertex = left;
                apex = left;
                apexIndex = leftIndex;
                left = apex;
                right = apex;
                leftIndex = apexIndex;
                rightIndex = apexIndex;

                // Portal index is capped at the real portal list size (goal
                // portal doesn't exist in the caller's data) for reporting.
                int reportIndex = (i < portals.size()) ? (int)i : (int)portals.size() - 1;
                outTrace.push_back({ reportIndex, apex, left, right, committed, committedVertex });

                i = apexIndex;
                continue;
            }
        }

        // --- update left side of the funnel ---
        if (Cross(apex, left, newLeft) >= 0.0f) {
            if ((apex.x == left.x && apex.y == left.y) || Cross(apex, right, newLeft) < 0.0f) {
                left = newLeft;
                leftIndex = i;
            }
            else {
                path.push_back(right);
                committed = true;
                committedVertex = right;
                apex = right;
                apexIndex = rightIndex;
                left = apex;
                right = apex;
                leftIndex = apexIndex;
                rightIndex = apexIndex;

                int reportIndex = (i < portals.size()) ? (int)i : (int)portals.size() - 1;
                outTrace.push_back({ reportIndex, apex, left, right, committed, committedVertex });

                i = apexIndex;
                continue;
            }
        }

        int reportIndex = (i < portals.size()) ? (int)i : (int)portals.size() - 1;
        outTrace.push_back({ reportIndex, apex, left, right, committed, committedVertex });
    }

    path.push_back(goal);
    return path;
}

std::vector<Vec2> RawPathFromPortals(const Vec2& start, const Vec2& goal, const std::vector<Portal>& portals) {
    std::vector<Vec2> path;
    path.push_back(start);
    for (const auto& p : portals) {
        path.push_back({ (p.left.x + p.right.x) * 0.5f, (p.left.y + p.right.y) * 0.5f });
    }
    path.push_back(goal);
    return path;
}

std::vector<Vec2> RubberBand(const Vec2& start, const Vec2& goal, const std::vector<Portal>& portals,
                              int iterations, float pullFactor) {
    std::vector<Vec2> path = RawPathFromPortals(start, goal, portals);

    for (int iter = 0; iter < iterations; ++iter) {
        for (size_t i = 1; i + 1 < path.size(); ++i) {
            Vec2 avg = {
                (path[i - 1].x + path[i + 1].x) * 0.5f,
                (path[i - 1].y + path[i + 1].y) * 0.5f
            };
            path[i].x += (avg.x - path[i].x) * pullFactor;
            path[i].y += (avg.y - path[i].y) * pullFactor;

            // Clamp back onto the corridor's portal segment (index i-1
            // corresponds to portals[i-1] since path[0] is start).
            if (i - 1 < portals.size()) {
                const Portal& port = portals[i - 1];
                float dx = port.right.x - port.left.x;
                float dy = port.right.y - port.left.y;
                float lenSq = dx * dx + dy * dy;
                if (lenSq > 0.0f) {
                    float t = ((path[i].x - port.left.x) * dx + (path[i].y - port.left.y) * dy) / lenSq;
                    if (t < 0.0f) t = 0.0f;
                    if (t > 1.0f) t = 1.0f;
                    path[i].x = port.left.x + dx * t;
                    path[i].y = port.left.y + dy * t;
                }
            }
        }
    }
    return path;
}

std::vector<Vec2> CatmullRom(const std::vector<Vec2>& raw, int samplesPerSegment) {
    std::vector<Vec2> curve;
    if (raw.size() < 2) return raw;

    // Duplicate first/last points so every real segment has 4 control points.
    std::vector<Vec2> pts;
    pts.push_back(raw.front());
    pts.insert(pts.end(), raw.begin(), raw.end());
    pts.push_back(raw.back());

    for (size_t i = 1; i + 2 < pts.size(); ++i) {
        const Vec2& p0 = pts[i - 1];
        const Vec2& p1 = pts[i];
        const Vec2& p2 = pts[i + 1];
        const Vec2& p3 = pts[i + 2];

        for (int s = 0; s < samplesPerSegment; ++s) {
            float t = (float)s / (float)samplesPerSegment;
            float t2 = t * t, t3 = t2 * t;
            float x = 0.5f * ((2.0f * p1.x) + (-p0.x + p2.x) * t +
                (2.0f * p0.x - 5.0f * p1.x + 4.0f * p2.x - p3.x) * t2 +
                (-p0.x + 3.0f * p1.x - 3.0f * p2.x + p3.x) * t3);
            float y = 0.5f * ((2.0f * p1.y) + (-p0.y + p2.y) * t +
                (2.0f * p0.y - 5.0f * p1.y + 4.0f * p2.y - p3.y) * t2 +
                (-p0.y + 3.0f * p1.y - 3.0f * p2.y + p3.y) * t3);
            curve.push_back({ x, y });
        }
    }
    curve.push_back(raw.back());
    return curve;
}