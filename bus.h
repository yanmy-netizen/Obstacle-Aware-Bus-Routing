#pragma once

#include "coordinates.h"
#include <vector>

using namespace std;

struct pin{
    int layername;
    rectangle r;
    string name;
    bool reverse;
    pin(int s, int a, int b, int c, int d, string name, bool reverse = false) : layername(s), r(a,b,c,d), name(name),reverse(reverse) {};
};

struct bus{
    int nb, np;
    vector<int> widths;
    vector<vector<pin> > pins;
    vector<bool> xsame;
    vector<bool> LSB;
    string name;
    
    bus(int nb, int np, vector<int> widths, vector<vector<pin> > pins, string name) : nb(nb), np(np), widths(widths),pins(pins),name(name) {
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