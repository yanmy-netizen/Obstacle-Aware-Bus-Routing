#pragma once

#include "coordinates.h"

struct obstacle{
    int layername;
    rectangle r;
    obstacle(int s, rectangle r):layername(s),r(r) {};
    obstacle(int s,int a, int b, int c, int d):layername(s), r(a,b,c,d) {};
};