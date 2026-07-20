#pragma once

struct Vec2 {
    float x, y;
};

// Cross product (z-component) of (b-a) x (c-a).
// >0 = c is left of line a->b, <0 = right, 0 = collinear.
inline float Cross(const Vec2& a, const Vec2& b, const Vec2& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}