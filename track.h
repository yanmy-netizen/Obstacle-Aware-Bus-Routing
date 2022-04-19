#pragma once

#include "coordinates.h"

struct track{
    int layername;
    line l;
    int spacing;
    track(int s, int a, int b , int c, int d, int spacing) : layername(s), l(a, b, c, d) ,spacing(spacing) {};
};