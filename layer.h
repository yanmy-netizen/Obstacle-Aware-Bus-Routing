#pragma once

#include "coordinates.h"

#include <string>
#include <set>

struct layer{
    int layername;
    string name;
    bool xsame;
    int spacing;
    set<pair<pair<int,int>, pair<int,int> > > used;
    set<pair<pair<int,int>, pair<int,int> > > temp_used;
    layer(int layername, string name, string s, int spacing) :layername(layername), name(name), spacing(spacing){
        if (s[0] == 'h') xsame = false;
        else xsame = true;
    }
    void add_use(pair<pair<int,int>, pair<int,int> > p) { 
        used.insert(p); 
        used.insert(make_pair(p.second, p.first));
    }
    void add_temp(pair<pair<int,int>, pair<int,int> > p) { 
        temp_used.insert(p); 
        temp_used.insert(make_pair(p.second, p.first));
    }
    bool can_use(pair<pair<int,int>, pair<int,int> > p) {
        if (p.first > p.second) p = make_pair(p.second, p.first); 
        auto it = used.lower_bound(make_pair(make_pair(p.first.first - spacing, p.first.second - spacing), make_pair(0,0)) );
        while (it != used.end()) {
            if (it->first.first <= p.first.first + spacing) {
                if (it->first.second <= p.second.second + spacing && it->first.second >= p.first.second - spacing)
                    return false;
                if (it->second.second <= p.second.second + spacing && it->second.second >= p.first.second - spacing)
                    return false;
                if ((it->first.second <= p.first.second - spacing)!= (it->second.second <= p.first.second - spacing))
                    return false;
                if ((it->first.second >= p.second.second + spacing)!= (it->second.second >= p.second.second + spacing)) 
                    return false;
            } else {
                break;
            }
            ++it;
        }
        it = temp_used.lower_bound(make_pair(make_pair(p.first.first - spacing, p.first.second - spacing), make_pair(0,0)) );
        while (it != temp_used.end()) {
            if (it->first.first <= p.first.first + spacing) {
                if (it->first.second <= p.second.second + spacing && it->first.second >= p.first.second - spacing)
                    return false;
                if (it->second.second <= p.second.second + spacing && it->second.second >= p.first.second - spacing)
                    return false;
                if ((it->first.second <= p.first.second - spacing)!= (it->second.second <= p.first.second - spacing))
                    return false;
                if ((it->first.second >= p.second.second + spacing)!= (it->second.second >= p.second.second + spacing)) 
                    return false;

            } else {
                break;
            }
            ++it;
        }
        return true;
    }
    void add_use(int a, int b, int c, int d) {
        add_use(make_pair(make_pair(a,b), make_pair(c,d)));
    }
    void add_temp(int a, int b, int c, int d) {
        add_temp(make_pair(make_pair(a,b), make_pair(c,d)));
    }
    bool can_use(int a, int b, int c, int d) {
        return can_use(make_pair(make_pair(a,b), make_pair(c,d)));
    }
    void clear_temp(int c = -1) {
        if (c == -1) {
            for (auto it = temp_used.begin(); it != temp_used.end(); ++it) {
                used.insert(*it);
            }
        }
        temp_used.clear();
    }
    void use_temp() {
        for (auto it = temp_used.begin(); it != temp_used.end(); ++it) {
            used.insert(*it);
        }
        temp_used.clear();
    }
    void clear_use() {
        used.clear();
    }
};