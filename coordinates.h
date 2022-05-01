#pragma once

#include "assert.h"

using namespace std;

struct point{
    int x;
    int y;
    point(int a = 0, int b = 0) {
        x = a;
        y = b;
    }
};

struct line{
    int x1, x2;
    int y1, y2;
    bool horizontal;
    bool xsame;
    line(int a = 0, int b = 0, int c = 0, int d = 0) {
        x1 = a;
        y1 = b;
        x2 = c;
        y2 = d;
        if (a != c) horizontal = true;
        else horizontal = false;
        xsame = !horizontal;
    }
};

pair<int, int> getCross(line l, line r) {
    assert(l.horizontal != r.horizontal);
    if (l.xsame) return make_pair(l.x1, r.y1);
    else return make_pair(r.x1, l.y1);
}

struct rectangle{
    int x1, x2;
    int y1, y2;
    rectangle(int a = 0, int b = 0, int c = 0, int d = 0) {
        x1 = a;
        y1 = b;
        x2 = c;
        y2 = d;
    }
};