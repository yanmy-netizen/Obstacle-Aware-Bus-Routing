#pragma once

#include "coordinates.h"
#include <vector>

struct track{
    int layername;
    line l;
    int spacing;
    vector<pair<int,int> > used;
    track(int s, int a, int b , int c, int d, int spacing) : layername(s), l(a, b, c, d) ,spacing(spacing) {};
    void use(pair<int,int> p) { used.push_back(p);}
    bool can_use(pair<int,int> p, int spacing = 0) {
        for (auto pa:used) {
            if (pa.first >= p.first - spacing && pa.first <= p.second + spacing) return false;
            if (pa.first >= p.second - spacing && pa.first <= p.first + spacing) return false;
            if (pa.second > p.first - spacing && pa.second < p.second + spacing) return false;
            if (pa.second > p.second - spacing && pa.second < p.first + spacing) return false;
        }
        return true;
    }
    void use(int i, int j) { used.push_back(make_pair(i,j));}
    bool can_use(int i, int j, int spacing = 0) {
        return can_use(make_pair(i, j));
    }
    void use(pair<int,int> p1, pair<int, int> p2) { 
        if (l.xsame) {
            assert(p1.first == p2.first);
            assert(p1.first == l.x1);
            use(make_pair(p1.second, p2.second));
        } else {
            assert(p1.second == p2.second);
            assert(p1.second == l.y1);
            use(make_pair(p1.first, p2.first));            
        }
    }
    bool can_use(pair<int,int> p1, pair<int, int> p2, int spacing = 0) {
        if (l.xsame) {
            assert(p1.first == p2.first);
            assert(p1.first == l.x1);
            return (can_use(make_pair(p1.second, p2.second), spacing));
        } else {
            assert(p1.second == p2.second);
            assert(p1.second == l.y1);
            return (can_use(make_pair(p1.first, p2.first), spacing));            
        }
    }
};