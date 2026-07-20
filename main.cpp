#include <cstdio>
#include <vector>

struct Vec2 {
    float x, y;
};

// Cross product (z-component) of (b-a) x (c-a).
// >0 = c is left of line a->b, <0 = right, 0 = collinear.
static float Cross(const Vec2& a, const Vec2& b, const Vec2& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// A portal is the shared edge between two triangles you cross while
// walking the raw path. left/right are that edge's endpoints, always
// given so "left" is on your left hand as you walk from start to goal.
struct Portal {
    Vec2 left;
    Vec2 right;
};

// Classic Simple Stupid Funnel Algorithm (Mikko Mononen's formulation).
// Input: start point, goal point, and the portals crossed in order.
// Output: the smoothed (taut) path through the corridor.
std::vector<Vec2> Funnel(const Vec2& start, const Vec2& goal, const std::vector<Portal>& portals) {
    std::vector<Vec2> path;
    path.push_back(start);

    Vec2 apex = start;
    Vec2 left = start;
    Vec2 right = start;
    size_t apexIndex = 0, leftIndex = 0, rightIndex = 0;

    // Append the goal as a zero-width final portal so the loop handles it uniformly.
    std::vector<Portal> pts = portals;
    pts.push_back({ goal, goal });

    for (size_t i = 1; i < pts.size(); ++i) {
        const Vec2& newLeft = pts[i].left;
        const Vec2& newRight = pts[i].right;

        // --- update right side of the funnel ---
        if (Cross(apex, right, newRight) <= 0.0f) {
            if ((apex.x == right.x && apex.y == right.y) || Cross(apex, left, newRight) > 0.0f) {
                right = newRight;
                rightIndex = i;
            }
            else {
                // Right crosses over left: apex moves to left, restart from there.
                path.push_back(left);
                apex = left;
                apexIndex = leftIndex;
                left = apex;
                right = apex;
                leftIndex = apexIndex;
                rightIndex = apexIndex;
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
                apex = right;
                apexIndex = rightIndex;
                left = apex;
                right = apex;
                leftIndex = apexIndex;
                rightIndex = apexIndex;
                i = apexIndex;
                continue;
            }
        }
    }

    path.push_back(goal);
    return path;
}

int main() {
    // Hand-built example corridor: a bent hallway made of a few triangles.
    // Replace this with real portals once your nav mesh / A* exists.
    Vec2 start = { 0.0f, 0.0f };
    Vec2 goal  = { 8.0f, 6.0f };

    std::vector<Portal> portals = {
        { {1.0f, 2.0f}, {1.0f, -1.0f} },
        { {3.0f, 3.0f}, {3.0f, 0.0f} },
        { {5.0f, 5.0f}, {5.0f, 1.0f} },
        { {7.0f, 7.0f}, {7.0f, 2.0f} },
    };

    std::vector<Vec2> smoothed = Funnel(start, goal, portals);

    printf("Smoothed path (%zu points):\n", smoothed.size());
    for (const auto& p : smoothed) {
        printf("  (%.2f, %.2f)\n", p.x, p.y);
    }

    return 0;
}