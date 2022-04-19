#pragma once

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
    line(int a = 0, int b = 0, int c = 0, int d = 0) {
        x1 = a;
        y1 = b;
        x2 = c;
        y2 = d;
        if (a != c) horizontal = true;
        else horizontal = false;
    }
};

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