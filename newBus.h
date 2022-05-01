#pragma once

#include "coordinates.h"
#include "bus.h"
#include <vector>

struct newBus{
    int nb, np, nf, cost;
    static int ab, ap, af;
    vector<int> widths;
    vector<vector<pin> > pins;
    vector<bool> xsame;
    vector<bool> LSB;
    newBus(vector<bus> v){
        nb = np = nf = 0;
        int s = v.size();
        np = v[0].np;
        widths = v[0].widths;
        xsame = v[0].xsame;
        LSB = v[0].LSB;
        for (int i = 0; i < s; i++) {
            nb += v[i].nb;
            for (auto a : v[i].pins) {
                pins.push_back(a);
            }
        }
        cost = nb * ab + np * ap;
    }
    void addnf() {
        ++nf;
        cost += af;
    }
    void resetnf() { 
        nf = 0;
        cost = nb * ab + np * ap;
    }
};