#pragma once

#include "coordinates.h"

#include <string>

struct layer{
    int layername;
    string name;
    bool horizontal;
    bool xsame;
    int spacing;
    layer(int layername, string name, bool horizontal, int spacing) :layername(layername), name(name), horizontal(horizontal), spacing(spacing){
        xsame = !horizontal;
    }
    layer(int layername, string name, string s, int spacing) :layername(layername), name(name), spacing(spacing){
        if (s[0] == 'h') horizontal = true;
        else horizontal = false;
        xsame = !horizontal;
    }
};