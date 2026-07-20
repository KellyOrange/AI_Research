#pragma once

struct Rect {
    float minX, minY, maxX, maxY;

    bool Overlaps(const Rect& other) const {
        return minX < other.maxX && maxX > other.minX &&
               minY < other.maxY && maxY > other.minY;
    }
};