#pragma once

#include "coordinates.h"
#include <vector>

using namespace std;

struct pin{
    int layername;
    rectangle r;
    pin(int s, int a, int b, int c, int d) : layername(s), r(a,b,c,d) {};
};

struct bus{
    int nb, np;
    vector<int> widths;
    vector<vector<pin> > pins;
    vector<bool> xsame;
    vector<bool> LSB;
    bus(int nb, int np, vector<int> widths, vector<vector<pin> > pins):nb(nb), np(np), widths(widths),pins(pins){
        int l = pins[0].size();
        xsame.resize(l);
        LSB.resize(l);
        for (int i =0; i < l; i++) {
            xsame[i] = (pins[0][i].r.x1 == pins[1][i].r.x1);
            if (xsame[i]) {
                LSB[i] = (pins[0][i].r.y1 < pins[1][i].r.y1);
            } else {
                LSB[i] = (pins[0][i].r.x1 < pins[1][i].r.x1);
            }
        }

    }
};