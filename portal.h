#pragma once

#include "Vec2.h"

// A portal is the shared edge between two triangles you cross while
// walking the raw path. left/right are that edge's endpoints, always
// given so "left" is on your left hand as you walk from start to goal.
struct Portal {
    Vec2 left;
    Vec2 right;
};