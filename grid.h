#pragma once

#include <stdlib.h>

struct grid{
    int l, t, x, y;
    int dl, dx, dy;
    int c,f;

    grid(){};
    grid(int l, int x, int y, int dl, int dx, int dy, int c):l(l), x(x), y(y), dl(dl), dx(dx), dy(dy), c(c) {
        int d = abs(x - dx) + abs(y - dy);
        d *= 1.1;
        int a = 0;
        if (x != dx) ++a; 
        if (y != dy) ++a;
        f = d + c + 1 * max(a - 1, abs(l-dl));
        if (x!= dx && y!= dy && l == dl) {
            f += 1;
        }
    }

};