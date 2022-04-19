#pragma once

#include "coordinates.h"

#include <string>

struct layer{
    int layername;
    string name;
    bool horizontal;
    int spacing;
    layer(int layername, string name, bool horizontal, int spacing) :layername(layername), name(name), horizontal(horizontal), spacing(spacing){};
    layer(int layername, string name, string s, int spacing) :layername(layername), name(name), spacing(spacing){
        if (s[0] == 'h') horizontal = true;
        else horizontal = false;
    };
};