#pragma once

#include "coordinates.h"

struct designBoundary{
    rectangle r;
    designBoundary(rectangle r1) :r(r1){}
    designBoundary(int a, int b, int c, int d) :r(a, b, c, d){}
};