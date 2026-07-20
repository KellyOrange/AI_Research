#include "Metrics.h"
#include <cstdio>
#include <cmath>

float PathLength(const std::vector<Vec2>& path) {
    float len = 0.0f;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        float dx = path[i + 1].x - path[i].x;
        float dy = path[i + 1].y - path[i].y;
        len += std::sqrt(dx * dx + dy * dy);
    }
    return len;
}

int TurnCount(const std::vector<Vec2>& path, float angleThresholdDeg) {
    if (path.size() < 3) return 0;
    int turns = 0;
    float thresholdRad = angleThresholdDeg * 3.14159265f / 180.0f;
    for (size_t i = 1; i + 1 < path.size(); ++i) {
        float ax = path[i].x - path[i - 1].x, ay = path[i].y - path[i - 1].y;
        float bx = path[i + 1].x - path[i].x, by = path[i + 1].y - path[i].y;
        float lenA = std::sqrt(ax * ax + ay * ay);
        float lenB = std::sqrt(bx * bx + by * by);
        if (lenA < 1e-6f || lenB < 1e-6f) continue;
        float dot = (ax * bx + ay * by) / (lenA * lenB);
        if (dot > 1.0f) dot = 1.0f;
        if (dot < -1.0f) dot = -1.0f;
        float angle = std::acos(dot);
        if (angle > thresholdRad) turns++;
    }
    return turns;
}

void PrintPath(const char* label, const std::vector<Vec2>& path) {
    printf("%s (%zu points):\n", label, path.size());
    for (const auto& p : path) {
        printf("  (%.2f, %.2f)\n", p.x, p.y);
    }
    printf("  length=%.3f  turns=%d\n\n", PathLength(path), TurnCount(path));
}