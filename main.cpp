#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <queue>
#include <iostream>
#include <cstring>
#include <algorithm>
#include <set>

#include "bus.h"
#include "coordinates.h"
#include "designBoundary.h"
#include "layer.h"
#include "obstacle.h"
#include "track.h"
#include "assert.h"
#include "newBus.h"
#include "grid.h"

int newBus::ab = 32;
int newBus::ap = 1;
int newBus::af = 8;

using namespace std;

enum direction {le, ri, up, down, front, back};
char numarr[10] = {'0','1','2','3','4','5','6','7','8','9'};

int mystoi(string s){
    int l = s.length();
    int r = 0;
    for (int i = 0; i < l; ++i) {
        if (s[i] >= '0' && s[i] <= '9') {
            r = r * 10 + s[i] - '0';
        }
    }
    return r;
}

template <typename T>
void printv(vector<T> v) {
    int s = v.size();
    for (int i = 0; i < s; i++) {
        cout << v[i] << ' ';
    }
    cout << endl;
}

template <typename T1, typename T2>
void printvp(vector<pair<T1, T2> >vp) {
    int s = vp.size();
    for (int i = 0; i < s; i++) {
        cout << vp[i].first << ' '<< vp[i].second << '\t';
    }
    cout << endl;
}

int runtime;
int alpha;
int beta;
int gamma;
int delta;
int epsilon;
int nlayers;
int ntracks;
int nbuses;
int nobstacles;

map<string, int> NameToLayer;
vector<layer> layers;
vector<track> tracks;
vector<vector<vector<track> > > dtracks;
vector<bus> buses;
vector<obstacle> obstacles;

vector<vector<int> > superBus;
vector<vector<vector<pair<int, int> > > > superBusPins;

vector<vector<int> > sBus;

map<pair<int, pair<int, int>>, vector<pair<int,pair<int, int>>>> final_path;

struct cmp1{
    bool operator()(const newBus l, const newBus r) const {
        return l.cost < r.cost;
    }
};
priority_queue<newBus, vector<newBus>, cmp1> newBuses;

struct cmp2{
    bool operator()(const grid l, const grid r) const {
        if (l.f != r.f) return l.f > r.f;
        return l.c < r.c;
    }
};

struct cmp3{
    bool operator()(const track l, const track r) const {
        if (l.l.x1 == r.l.x1 && l.l.x2 == r.l.x2) return l.l.y1 < r.l.y1;
        if (l.l.x1 == r.l.x1) return l.l.x2 < r.l.x2;
        return l.l.x1 < r.l.x1;
    }
};

struct cmp4{
    bool operator()(const track l, const track r) const {
        if (l.l.y1 == r.l.y1 && l.l.y2 == r.l.y2) return l.l.x1 < r.l.x1;
        if (l.l.y1 == r.l.y1) return l.l.y2 < r.l.y2;
        return l.l.y1 < r.l.y1;
    }
};

struct cmp5{
    bool operator()(const track l, const track r) const {
        if (l.l.y1 != r.l.y1) return l.l.y1 < r.l.y1;
        if (l.l.x1 == r.l.x1) return l.l.x2 < r.l.x2;
        return l.l.x1 < r.l.x1;
    }
};

struct cmp6{
    bool operator()(const track l, const track r) const {
        if (l.l.x1 != r.l.x1) return l.l.x1 < r.l.x1;
        if (l.l.y1 == r.l.y1) return l.l.y2 < r.l.y2;
        return l.l.y1 < r.l.y1;
    }
};

// struct cmp3{
//     bool operator()(const track l, const track r) const {
//         if (l.l.x1 == r.l.x1 || l.l.x2 == r.l.x2) return l.l.y1 < r.l.y1;
//         if (l.l.x1 == r.l.x1) return l.l.x2 < r.l.x2;
//         return l.l.x1 < r.l.x1;
//     }
// };

// struct cmp4{
//     bool operator()(const track l, const track r) const {
//         if (l.l.y1 == r.l.y1 || l.l.y2 == r.l.y2) return l.l.x1 < r.l.x1;
//         if (l.l.y1 == r.l.y1) return l.l.y2 < r.l.y2;
//         return l.l.y1 < r.l.y1;
//     }
// };

void readInput () {
    string s;
    string ss[5];
    string sss[5];
    int l, m;
    cin >> s >> runtime;
    cin >> s >> alpha;
    cin >> s >> beta;
    cin >> s >> gamma;
    cin >> s >> delta;
    cin >> s >> epsilon;
    cin >> ss[0] >> ss[1] >> ss[2] >> ss[3] >> ss[4];
    designBoundary db(mystoi(ss[1]), mystoi(ss[2]), mystoi(ss[3]), mystoi(ss[4]));
    // snumx.insert(mystoi(ss[1]));
    // snumy.insert(mystoi(ss[2]));
    // snumx.insert(mystoi(ss[3]));
    // snumy.insert(mystoi(ss[4]));

    cin >> s >> nlayers;
    for (int i = 0; i < nlayers; ++i) {
        cin >> ss[0] >> ss[1] >> m;
        NameToLayer[ss[0]] = i;
        layers.push_back(layer(i, ss[0], ss[1], m));
    }
    cin >> s;

    cin >> s >> ntracks;
    int ct = -1;
    int lx1, ly1, lx2, ly2;
    vector<vector<track> > tdtracks;
    vector<track> odtracks;
    vector<track> oodtracks;
    for (int i = 0; i < ntracks; ++i) {
        cin >> ss[0] >> ss[1] >> ss[2] >> ss[3] >> ss[4] >> m;
        
        tracks.push_back(track(NameToLayer[ss[0]], mystoi(ss[1]), mystoi(ss[2]), mystoi(ss[3]), mystoi(ss[4]), m));
        // snumx.insert(mystoi(ss[1]));
        // snumy.insert(mystoi(ss[2]));
        // snumx.insert(mystoi(ss[3]));
        // snumy.insert(mystoi(ss[4]));
        int tt = NameToLayer[ss[0]];
        int tx1 = mystoi(ss[1]);
        int ty1 = mystoi(ss[2]);
        int tx2 = mystoi(ss[3]);
        int ty2 = mystoi(ss[4]);
        if (ct != tt) {
            
            if (ct != -1) {
                int cx1, cx2, cy1, cy2, ttx1, tty1, ttx2, tty2;
                if (layers[ct].xsame)
                    sort(odtracks.begin(), odtracks.end(), cmp4());
                else
                    sort(odtracks.begin(), odtracks.end(), cmp3());
                int os = odtracks.size();
                for (int ij = 0; ij < os; ij++) {
                    ttx1 = odtracks[ij].l.x1;
                    ttx2 = odtracks[ij].l.x2;
                    tty1 = odtracks[ij].l.y1;
                    tty2 = odtracks[ij].l.y2;
                    if (ij > 0 && ((layers[ct].xsame && (tty1 != cy1 || tty2 != cy2)) || (!layers[ct].xsame && (ttx1 != cx1 || ttx2 != cx2)))) {
                            // cout << ttx1 << " " << tty1 << " " << ttx2 << " " << tty2 << endl;
                            // cout << cx1 << " " << cy1 << " " << cx2 << " " << cy2 << endl;
                        tdtracks.push_back(oodtracks);
                        oodtracks.clear();
                    }
                    oodtracks.push_back(odtracks[ij]);
                    cx1 = ttx1;
                    cx2 = ttx2;
                    cy1 = tty1;
                    cy2 = tty2;
                }
                tdtracks.push_back(oodtracks);
                oodtracks.clear();
                dtracks.push_back(tdtracks);
                tdtracks.clear();
                odtracks.clear();
                // cout << endl << endl;
            }
        }
        ct = tt;
        lx1 = tx1;
        ly1 = ty1;
        lx2 = tx2;
        ly2 = ty2;
        odtracks.push_back(track(NameToLayer[ss[0]], mystoi(ss[1]), mystoi(ss[2]), mystoi(ss[3]), mystoi(ss[4]), m));
    }

    int cx1, cx2, cy1, cy2, ttx1, tty1, ttx2, tty2;
    if (layers[ct].xsame)
        sort(odtracks.begin(), odtracks.end(), cmp4());
    else
        sort(odtracks.begin(), odtracks.end(), cmp3());
    int os = odtracks.size();
    for (int ij = 0; ij < os; ij++) {
        ttx1 = odtracks[ij].l.x1;
        ttx2 = odtracks[ij].l.x2;
        tty1 = odtracks[ij].l.y1;
        tty2 = odtracks[ij].l.y2;
        if (ij > 0 && ((layers[ct].xsame && (tty1 != cy1 || tty2 != cy2)) || (!layers[ct].xsame && (ttx1 != cx1 || ttx2 != cx2)))) {
            // cout << ttx1 << " " << tty1 << " " << ttx2 << " " << tty2 << endl;
            // cout << cx1 << " " << cy1 << " " << cx2 << " " << cy2 << endl;
            tdtracks.push_back(oodtracks);
            oodtracks.clear();
        }
        oodtracks.push_back(odtracks[ij]);
        cx1 = ttx1;
        cx2 = ttx2;
        cy1 = tty1;
        cy2 = tty2;
    }
    tdtracks.push_back(oodtracks);
    oodtracks.clear();
    dtracks.push_back(tdtracks);
    tdtracks.clear();
    odtracks.clear();

    cin >> s;

    cin >> s >> nbuses;
    for (int i = 0; i < nbuses; ++i) {
        string busname;
        int nb, np, nw, ww;
        cin >> s >> busname >> nb >> np;
        cin >> s >> nw;
        assert(np == 2);
        vector<int> nwv(nw);
        for (int j = 0; j < nw; ++j) {
            cin >> ww;
            nwv[j] = ww;
        }
        cin >> s;
        vector<vector<pin> > pinss;
        for (int j = 0; j < nb; ++j) {
            string pinname;
            cin >> s >> pinname;
            {
                vector<pin> pins;
                cin >> ss[0] >> ss[1] >> ss[2] >> ss[3] >> ss[4];
                cin >> sss[0] >> sss[1] >> sss[2] >> sss[3] >> sss[4];
                if (NameToLayer[ss[0]] < NameToLayer[sss[0]]) {
                    pins.push_back(pin(NameToLayer[ss[0]], mystoi(ss[1]), mystoi(ss[2]), mystoi(ss[3]), mystoi(ss[4]), pinname));
                    pins.push_back(pin(NameToLayer[sss[0]], mystoi(sss[1]), mystoi(sss[2]), mystoi(sss[3]), mystoi(sss[4]), pinname));
                } else if (NameToLayer[ss[0]] > NameToLayer[sss[0]]) {
                    pins.push_back(pin(NameToLayer[sss[0]], mystoi(sss[1]), mystoi(sss[2]), mystoi(sss[3]), mystoi(sss[4]), pinname, true));
                    pins.push_back(pin(NameToLayer[ss[0]], mystoi(ss[1]), mystoi(ss[2]), mystoi(ss[3]), mystoi(ss[4]), pinname, true));
                } else if (mystoi(ss[1]) < mystoi(sss[1]) || (mystoi(ss[1]) == mystoi(sss[1]) && mystoi(ss[2]) < mystoi(sss[2]))) {
                    pins.push_back(pin(NameToLayer[ss[0]], mystoi(ss[1]), mystoi(ss[2]), mystoi(ss[3]), mystoi(ss[4]), pinname));
                    pins.push_back(pin(NameToLayer[sss[0]], mystoi(sss[1]), mystoi(sss[2]), mystoi(sss[3]), mystoi(sss[4]), pinname));                    
                } else {
                    pins.push_back(pin(NameToLayer[sss[0]], mystoi(sss[1]), mystoi(sss[2]), mystoi(sss[3]), mystoi(sss[4]), pinname, true));
                    pins.push_back(pin(NameToLayer[ss[0]], mystoi(ss[1]), mystoi(ss[2]), mystoi(ss[3]), mystoi(ss[4]), pinname, true));                    
                }
                pinss.push_back(pins);
                    // snumx.insert(mystoi(ss[1]));
                    // snumy.insert(mystoi(ss[2]));
                    // snumx.insert(mystoi(ss[3]));
                    // snumy.insert(mystoi(ss[4]));
                    // snumx.insert(mystoi(sss[1]));
                    // snumy.insert(mystoi(sss[2]));
                    // snumx.insert(mystoi(sss[3]));
                    // snumy.insert(mystoi(sss[4]));
            }
            cin >> s;
        }
        buses.push_back(bus(nb, np, nwv, pinss, busname));
        cin >> s;
    }
    cin >> s;

    cin >> s >> nobstacles;
    for (int i = 0; i < nobstacles; ++i) {
        cin >> ss[0] >> ss[1] >> ss[2] >> ss[3] >> ss[4];
        obstacles.push_back(obstacle(NameToLayer[ss[0]], mystoi(ss[1]), mystoi(ss[2]), mystoi(ss[3]), mystoi(ss[4])));
        int la = NameToLayer[ss[0]];
        int x1 = mystoi(ss[1]);
        int y1 = mystoi(ss[2]);
        int x2 = mystoi(ss[3]);
        int y2 = mystoi(ss[4]);
        int spacing = layers[la].spacing;

        if (layers[l].xsame) {
            int start = x1 - 2*spacing;
            while (start < x2) {
                start += 2 * spacing;
                if (start > x2) start = x2;
                layers[la].add_use(start, y1, start, y2);
            }
        } else {
            int start = y1 - 2*spacing;
            while (start < y2) {
                start += 2 * spacing;
                if (start > y2) start = y2;
                layers[la].add_use(x1, start, x2, start);
            }
        }
        // snumx.insert(mystoi(ss[1]));
        // snumy.insert(mystoi(ss[2]));
        // snumx.insert(mystoi(ss[3]));
        // snumy.insert(mystoi(ss[4]));
    }
    cin >> s;
    // for (auto it = snumx.begin(); it != snumx.end(); ++it) {
    //     numx.push_back(*it);
    // }
    // for (auto it = snumy.begin(); it != snumy.end(); ++it) {
    //     numy.push_back(*it);
    // }
    // cout << numx.size() << " " << numy.size() << endl;
    // snumx.clear();
    // snumy.clear();
}

void getSuperTrack() {
    int d = dtracks.size();
    for (int k = 0; k < d; ++k) {
        auto &trs = dtracks[k];
        int l = trs.size();
        bool* b = new bool[l*l];
        memset(b, 0, sizeof(bool)*l*l);
        vector<vector<track> > cts;
        vector<track> ts;
        if (layers[k].xsame) {
            for (int i = 0; i < l; ++i) {
                for(int j = 0; j < l; ++j) {
                    if (trs[j][0].l.y1 <= trs[i][0].l.y1 && trs[j][0].l.y2 >= trs[i][0].l.y2){
                        ts.insert(ts.end(), trs[j].begin(), trs[j].end());
                    }
                }
                sort(ts.begin(), ts.end(), cmp6());
                cts.push_back(ts);
                ts.clear();
            }
            
        } else {
            for (int i = 0; i < l; ++i) {
                for(int j = 0; j < l; ++j) {
                    if (trs[j][0].l.x1 <= trs[i][0].l.x1 && trs[j][0].l.x2 >= trs[i][0].l.x2){
                        ts.insert(ts.end(), trs[j].begin(), trs[j].end());
                    }
                }
                sort(ts.begin(), ts.end(), cmp5());
                cts.push_back(ts);
                ts.clear();
            }
            
        }
        trs = cts;
        cts.clear();

    }
}

bool isSuperBus (bus b1, bus b2) {
    int nw = b1.widths.size();
    for (int i = 0; i < nw; ++i) {
        if (b1.widths[i] != b2.widths[i]) {
            return false;
        }
    }
    int b1l0 = b1.pins[0][0].layername;
    int b1l1 = b1.pins[0][1].layername;
    int b2l0 = b2.pins[0][0].layername;
    int b2l1 = b2.pins[0][1].layername;

    if (b1l0 == b2l0 && b1l1 == b2l1) {
        if (b1.pins[0][0].r.x1 == b1.pins[1][0].r.x1) {
            if (b1.pins[0][0].r.x1 != b2.pins[0][0].r.x1) return false;
            if (b1.pins[0][0].r.x2 != b2.pins[0][0].r.x2) return false;
        } else {
            if (b1.pins[0][0].r.y1 != b2.pins[0][0].r.y1) return false;
            if (b1.pins[0][0].r.y2 != b2.pins[0][0].r.y2) return false;            
        }

        if (b1.pins[0][1].r.x1 == b1.pins[1][1].r.x1) {
            if (b1.pins[0][1].r.x1 != b2.pins[0][1].r.x1) return false;
            if (b1.pins[0][1].r.x2 != b2.pins[0][1].r.x2) return false;
        } else {
            if (b1.pins[0][1].r.y1 != b2.pins[0][1].r.y1) return false;
            if (b1.pins[0][1].r.y2 != b2.pins[0][1].r.y2) return false;            
        }

        return true;
    } else return false;
}

void getSuperBus () {
    int l = buses.size();
    vector<int> p(l, 0);
    vector<int> pa;
    for (int i = 0; i < l; ++i) {
        int k = pa.size();
        bool found = false;
        for (int j = 0; j < k; ++j) {
            if (isSuperBus(buses[i], buses[pa[j]])) {
                p[i] = j;
                found = true;
                break;
            }
        }
        if (!found) {
            pa.push_back(i);
            p[i] = k;
        }
    }
    int k = pa.size();
    superBus.resize(k);

    for (int i = 0; i < l; ++i) {
        superBus[p[i]].push_back(i);
    }
}

vector<pair<int, int> > getSuperBusPins1 (int i, int k) {
    vector<pair<int, int> > ret;
    int li = superBus[i].size();
    
    vector<int> lij(li);
    int sum = 0;
    for (int j = 0; j < li; ++j) {
        lij[j] = buses[superBus[i][j]].pins.size();
        sum += lij[j];
    }
    int cur = 0;
    priority_queue<pair<int, pair<int, int> >, vector<pair<int, pair<int, int> > >, greater<pair<int, pair<int, int> > > >pq;
    if (buses[superBus[i][0]].xsame[k]) {
        vector<int> curij(li);
        vector<int> endij(li);

        for (int j = 0; j < li; ++j) {
            if (buses[superBus[i][0]].LSB[k]) {
                curij[j] = 0;
                endij[j] = lij[j];
            } else {
                curij[j] = lij[j] - 1;
                endij[j] = -1;                
            }
            pq.push(make_pair(buses[superBus[i][j]].pins[curij[j]][k].r.y1, make_pair(j, curij[j])));
        }
        while (cur < sum) {   
            auto p = pq.top();
            pq.pop();
            ret.push_back(make_pair(superBus[i][p.second.first], p.second.second));
            int cj = p.second.first;
            if (curij[cj] < endij[cj]) {
                ++curij[cj];
            } else {
                --curij[cj];
            }
            ++cur;
            if (curij[cj] != endij[cj]) {
                pq.push(make_pair(buses[superBus[i][cj]].pins[curij[cj]][k].r.y1, make_pair(cj, curij[cj])));
            }
        }
    } else {
        vector<int> curij(li);
        vector<int> endij(li);

        for (int j = 0; j < li; ++j) {
            if (buses[superBus[i][0]].LSB[k]) {
                curij[j] = 0;
                endij[j] = lij[j];
            } else {
                curij[j] = lij[j] - 1;
                endij[j] = -1;                
            }
            pq.push(make_pair(buses[superBus[i][j]].pins[curij[j]][k].r.x1, make_pair(j, curij[j])));
        }
        while (cur < sum) {   
            auto p = pq.top();
            pq.pop();
            ret.push_back(make_pair(superBus[i][p.second.first], p.second.second));
            int cj = p.second.first;
            if (curij[cj] < endij[cj]) {
                ++curij[cj];
            } else {
                --curij[cj];
            }
            ++cur;
            if (curij[cj] != endij[cj]) {
                pq.push(make_pair(buses[superBus[i][cj]].pins[curij[cj]][k].r.x1, make_pair(cj, curij[cj])));
            }
        } 
    }
    return ret;
}

vector<vector<pair<int, int> > > getSuperBusPins2 (int i) {
    vector<vector<pair<int, int> > > ret;
    ret.push_back(getSuperBusPins1(i, 0));
    ret.push_back(getSuperBusPins1(i, 1));
    return ret;
}

void getSuperBusPins3() {
    int k = superBus.size();
    for (int i = 0; i < k; ++i) {
        superBusPins.push_back(getSuperBusPins2(i));
    }
}

template <typename T>
vector<T> LCS(const vector<T>& l, const vector<T>& r) {
    int ll = l.size();
    int lr = r.size();
    vector<vector<int> > f(ll+1, vector<int>(lr+1, 0));
    for (int i = 1; i <= ll; ++i) {
        for (int j = 1; j <= lr; ++j) {
            if (l[i-1] == r[j-1]) {
                f[i][j] = f[i-1][j-1] + 1;
            } else {
                f[i][j] = max(f[i-1][j], f[i][j-1]);
            }
        }
    }
    int cl = ll;
    int cr = lr;
    int c = f[cl][cr];
    vector<T> ret(c);
    while(c > 0) {
        while(f[cl-1][cr] == c) --cl;
        while(f[cl][cr-1] == c) --cr;
        --c;
        ret[c] = l[cl-1];
        --cl;
        --cr;
    }
    return ret;
}

vector<int> getRemoveBus (vector<pair<int, int> > vp) {
    int* c = new int[nbuses];
    memset(c, 0, nbuses*sizeof(int));
    vector<int> ret;
    for (auto & a : vp) {
        ++c[a.first];
    }
    for (int i = 0; i < nbuses; ++i) {
        if (c[i] == buses[i].pins.size()) {
            ret.push_back(i);
        }
    }
    return ret;
}

vector<pair<int, int> > removeBus(vector<int> v, vector<pair<int, int> > vp) {
    int s = v.size();
    bool* b = new bool[nbuses];
    memset(b, 0, nbuses*sizeof(bool));
    for (int i = 0; i < s; ++i) {
        b[v[i]] = true;
    }
    vector<pair<int, int> > ret;
    for (auto & p : vp) {
        if (!b[p.first]) {
            ret.push_back(p);
        }
    }
    return ret;
}

vector<int> extractBuses(vector<pair<int, int> >& l, vector<pair<int, int> >& r) {
    vector<pair<int, int> > lcs1 = LCS(l, r);
    // printvp(lcs1);
    reverse(r.begin(), r.end());
    vector<pair<int, int> > lcs2 = LCS(l, r);
    // printvp(lcs2);
    vector<int> ret;
    if (lcs1.size() < lcs2.size()) {
        ret = getRemoveBus(lcs2);
    } else {
        ret = getRemoveBus(lcs1);
        reverse(r.begin(), r.end());
    }
    l = removeBus(ret, l);
    r = removeBus(ret, r);
    return ret;
}

void getSBus() {
    int k = superBusPins.size();
    bool* b = new bool[nbuses];
    memset(b, 0, nbuses * sizeof(bool));
    for (int i = 0; i <k; ++i) {
        auto v1 = superBusPins[i][0];
        auto v2 = superBusPins[i][1];
        // printvp(v1);
        // printvp(v2);
        // while (1) {
        //     vector<int> v = extractBuses(v1, v2);
        // //     printv(v);
        //     if (v.size() == 0) break;
        //     sBus.push_back(v);
        //     for (auto & a : v) {
        //         b[a] = true;
        //     }
        // }
        for (auto & k : v1) {
            if(!b[k.first]) {
                b[k.first] = true;
                sBus.push_back(vector<int>(1,k.first));
            }
        }
    }
}

void getnewBuses() {
    int c = 0;
    for (auto a : sBus) {
        // cout << ++c << endl;
        vector<bus> b;
        for (auto i : a) {
            b.push_back(buses[i]);
        }
        newBuses.push(newBus(b));
    }
}

int binary_search_x(vector<track> & nums, int target) {
    int l = 0;
    int r = nums.size() - 1;
    while (l <= r) {
        int m = (l + r) / 2;
        if (nums[m].l.x1 == target){
            while(m > 0 && nums[m-1].l.x1 == target) {
                --m;
            }
            return m;
        }
        else if (nums[m].l.x1 > target)
            r = m - 1;
        else
            l = m + 1;
    }
    return -1;
}

pair<int,int> binary_search_x(vector<track> & nums, int target1, int target2) {
    int l = 0;
    int r = nums.size() - 1;
    int ret;
    bool found = false;
    while (l <= r) {
        int m = (l + r) / 2;
        if (nums[m].l.x1 == target1) {
            ret = m;
            found = true;
            break;
        }
        else if (nums[m].l.x1 > target1)
            r = m - 1;
        else
            l = m + 1;
    }
    if (!found) ret = l;
    l = 0;
    r = nums.size() - 1;
    while (l <= r) {
        int m = (l + r) / 2;
        if (nums[m].l.x1 == target2)
            return make_pair(ret, m);
        else if (nums[m].l.x1 > target2)
            r = m - 1;
        else
            l = m + 1;
    }
    return make_pair(ret, r);    
}

int binary_search_y(vector<track> & nums, int target) {
    int l = 0;
    int r = nums.size() - 1;
    while (l <= r) {
        int m = (l + r) / 2;      
        if (nums[m].l.y1 == target){
            while(m > 0 && nums[m-1].l.y1 == target) {
                --m;
            }
            return m;
        }
        else if (nums[m].l.y1 > target)
            r = m - 1;
        else
            l = m + 1;
    }
    return -1;
}

pair<int,int> binary_search_y(vector<track> & nums, int target1, int target2) {
    int l = 0;
    int r = nums.size() - 1;
    int ret;
    bool found = false;
    while (l <= r) {
        int m = (l + r) / 2;
        if (nums[m].l.y1 == target1) {
            ret = m;
            found = true;
            break;
        }
        else if (nums[m].l.y1 > target1)
            r = m - 1;
        else
            l = m + 1;
    }
    if (!found) ret = l;
    l = 0;
    r = nums.size() - 1;
    while (l <= r) {
        int m = (l + r) / 2;
        if (nums[m].l.y1== target2)
            return make_pair(ret, m);
        else if (nums[m].l.y1 > target2)
            r = m - 1;
        else
            l = m + 1;
    }
    return make_pair(ret, r);    
}

int getRelatedTracks(int l, int x, int y) {
    if (layers[l].xsame) {
        for (auto &ts : dtracks[l]) {
            if (ts[0].l.y1 > y || ts[0].l.y2 < y) {
                continue;
            }
            int b = binary_search_x(ts, x);
            if (b != -1)
                return b;
        }
    } else {
        for (auto &ts : dtracks[l]) {
            if (ts[0].l.x1 > x || ts[0].l.x2 < x) {
                continue;
            }
            int b = binary_search_y(ts, y);
            if (b != -1)
                return b;
        }        
    }
    return -1;
}

vector<track> getRelatedTracks(int l, int x1, int y1, int x2, int y2) {
    vector<track> ret;
    if (layers[l].xsame) {
        for (auto &ts : dtracks[l]) {
            if (ts[0].l.y1 > y2 || ts[0].l.y2 < y1) {
                continue;
            }
            auto p = binary_search_x(ts, x1, x2);
            for (int i = p.first; i <= p.second; ++i) {
                ret.push_back(ts[i]);
            }
        }
    } else {
        for (auto &ts : dtracks[l]) {
            if (ts[0].l.x1 > x2 || ts[0].l.x2 < x1) {
                continue;
            }
            auto p = binary_search_y(ts, y1, y2);
            for (int i = p.first; i <= p.second; ++i) {
                ret.push_back(ts[i]);
            }
        }        
    }
    return ret;
}

vector<track> findTrack(grid g) {
    vector<track> ret;
    int l = g.l;
    int x = g.x;
    int y = g.y;
    if (layers[l].xsame) {
        for (auto &ts : dtracks[l]) {
            if (ts[0].l.y1 > y || ts[0].l.y2 < y) {
                continue;
            }
            int b = binary_search_x(ts, x);
            if (b != -1)
                ret.push_back(ts[b]);
        }
    } else {
        for (auto &ts : dtracks[l]) {
            if (ts[0].l.x1 > x || ts[0].l.x2 < x) {
                continue;
            }
            int b = binary_search_y(ts, y);
            if (b != -1)
                ret.push_back(ts[b]);
        }        
    }
    return ret;
}

vector<track> findTrack(int l, int x, int y) {
    vector<track> ret;
    if (layers[l].xsame) {
        for (auto &ts : dtracks[l]) {
            if (ts[0].l.y1 > y || ts[0].l.y2 < y) {
                continue;
            }
            int b = binary_search_x(ts, x);
            if (b != -1)
                ret.push_back(ts[b]);
        }
    } else {
        for (auto &ts : dtracks[l]) {
            if (ts[0].l.x1 > x || ts[0].l.x2 < x) {
                continue;
            }
            int b = binary_search_y(ts, y);
            if (b != -1)
                ret.push_back(ts[b]);
        }        
    }
    return ret;
}

tuple<int, int, int> findTrackIdx(grid g) {
    int l = g.l;
    int x = g.x;
    int y = g.y;
    if (layers[l].xsame) {
        int s = dtracks[l].size();
        for (int i = 0; i < s; ++i) {
            auto &ts = dtracks[l][i];
            if (ts[0].l.y1 > y || ts[0].l.y2 < y) {
                continue;
            }
            int b = binary_search_x(ts, x);
            if (b != -1)
                return make_tuple(l, i, b);
        }
    } else {
        int s = dtracks[l].size();
        for (int i = 0; i < s; ++i) {
            auto &ts = dtracks[l][i];
            if (ts[0].l.x1 > x || ts[0].l.x2 < x) {
                continue;
            }
            int b = binary_search_y(ts, y);
            if (b != -1)
                return make_tuple(l, i, b);
        }        
    }
    assert(1 == 0);    
}

tuple<int, int, int> findTrackIdx(int l, int x, int y) {
    if (layers[l].xsame) {
        int s = dtracks[l].size();
        for (int i = 0; i < s; ++i) {
            auto &ts = dtracks[l][i];
            if (ts[0].l.y1 > y || ts[0].l.y2 < y) {
                continue;
            }
            int b = binary_search_x(ts, x);
            if (b != -1)
                return make_tuple(l, i, b);
        }
    } else {
        int s = dtracks[l].size();
        for (int i = 0; i < s; ++i) {
            auto &ts = dtracks[l][i];
            if (ts[0].l.x1 > x || ts[0].l.x2 < x) {
                continue;
            }
            int b = binary_search_y(ts, y);
            if (b != -1)
                return make_tuple(l, i, b);
        }        
    }
    assert(1 == 0);    
}

vector<grid> getRelatedGrids(grid g) {
    vector<grid> ret;
    auto tl = findTrack(g);
    for (auto &t : tl) {
        if (g.x != t.l.x1 || g.y != t.l.y1)
            ret.push_back(grid(g.l, t.l.x1, t.l.y1, g.dl, g.dx, g.dy, g.c+abs(g.x - t.l.x1) + abs(g.y - t.l.y1)));
        if (g.x != t.l.x2 || g.y != t.l.y2)
            ret.push_back(grid(g.l, t.l.x2, t.l.y2, g.dl, g.dx, g.dy, g.c+abs(g.x - t.l.x2) + abs(g.y - t.l.y2)));
        if (g.l > 0) {
            auto a = getRelatedTracks(g.l-1, g.x, g.y);
            if (a!= -1) {
                ret.push_back(grid(g.l-1, g.x, g.y, g.dl, g.dx, g.dy, g.c+1));
            }
            auto aa = getRelatedTracks(g.l-1, t.l.x1, t.l.y1, t.l.x2, t.l.y2);
            if (!aa.empty()) {
                for (auto aaa: aa) {
                    auto p = getCross(aaa.l, t.l);
                    ret.push_back(grid(g.l, p.first, p.second, g.dl, g.dx, g.dy, g.c + abs(g.x - p.first) + abs(g.y - p.second)));
                }
            }
        }
        if (g.l != layers.size() - 1) {
            auto a = getRelatedTracks(g.l+1, g.x, g.y);
            if (a!= -1) {
                ret.push_back(grid(g.l+1, g.x, g.y, g.dl, g.dx, g.dy, g.c+1));
                // cout << g.l + 1 << g.x << g.y << endl;
                // auto ft = findTrack(grid(g.l+1, g.x, g.y, g.dl, g.dx, g.dy, g.c+1, &g));
                // cout << ft.l.x1 << ' ' << ft.l.y1 << ' ' << ft.l.x2 << ' ' << ft.l.y2 <<endl;
            }
            auto aa = getRelatedTracks(g.l+1, t.l.x1, t.l.y1, t.l.x2, t.l.y2);
            if (!aa.empty()) {
                for (auto aaa: aa) {
                    auto p = getCross(aaa.l, t.l);
                    ret.push_back(grid(g.l, p.first, p.second, g.dl, g.dx, g.dy, g.c + abs(g.x - p.first) + abs(g.y - p.second)));
                }
            }
        }
    }
    

    return ret;
    
}

vector<pair<int,pair<int, int> > > astar(grid a, bool areverse = false) {
    // cout << "--------------------------------" << endl;
    vector<pair<int,pair<int, int> > > ret;
    priority_queue<grid, vector<grid>, cmp2> p;
    map<pair<int, pair<int,int> >, pair<int, pair<int,int> > > s;
    p.push(a);
    auto ter = make_pair(-1,make_pair(-1,-1));
    s[make_pair(a.l,make_pair(a.x,a.y))] = ter;
    pair<int,pair<int,int> >cur;
    bool found = false;
    for (int k = 0; k < 2000; ++k) {
        assert(!p.empty());
        grid t = p.top();
        auto tp = make_pair(t.l,make_pair(t.x,t.y));
        p.pop();
        // cout <<t.l << " " << t.x << " " << t.y << " " << t.c << " ";
        // cout << t.dl << " " << t.dx << " " << t.dy << " " << t.f<<  endl;
        auto vg = getRelatedGrids(t);
        for (auto gr: vg) {
            auto pa = make_pair(gr.l, make_pair(gr.x,gr.y));
            // if (!layers[gr.l].can_use(t.x, t.y, gr.x, gr.y)) {
            //     continue;
            // }
            if (gr.x == a.x && gr.y == a.y) continue;
            if (gr.l == gr.dl && gr.x == gr.dx && gr.y == gr.dy) {
                s[pa] = tp;
                cur = pa;
                found = true;
                break;
            }
            if (gr.x == a.dx && gr.y == a.dy) continue;
            
            if (s.find(pa) == s.end()) {
                s[pa] = tp;
                p.push(gr);
            }
        }
        if (found) break;
        if (areverse) --k;
    }
    if (!found) {
        assert(!areverse);
        auto vpp = astar(grid(a.dl, a.dx, a.dy, a.l, a.x, a.y, a.c), true);
        reverse(vpp.begin(),vpp.end());
        return vpp;
    }
    while(cur != ter) {
        ret.push_back(cur);
        cur = s[cur];
    }
    reverse(ret.begin(), ret.end());
    return ret;
}

vector<int> randomk (int k, int n) {
    vector<int> temp;
    for (int i = 0; i < n; ++i)
    {
        temp.push_back(i);
    }
    random_shuffle(temp.begin(), temp.end());
    temp.resize(k);
    return temp;
}

pair<int,int> findPinxy (pin p) {
    int l = p.layername;
    if (layers[l].xsame) {
        int x = (p.r.x1 + p.r.x2) / 2;
        
        for (auto &ts : dtracks[l]) {
            if (ts[0].l.y1 != p.r.y1 && ts[0].l.y2 != p.r.y2) {
                continue;
            }
            int b = binary_search_x(ts, x);
            if (b != -1) {
                if (ts[0].l.y1 == p.r.y1) {
                    return make_pair(x, p.r.y1);
                } else {
                    return make_pair(x, p.r.y2);
                }
            }
        }
    } else {
        int y = (p.r.y1 + p.r.y2) / 2;
        for (auto &ts : dtracks[l]) {
            if (ts[0].l.x1 != p.r.x1 && ts[0].l.x2 != p.r.x2) {
                continue;
            }
            int b = binary_search_y(ts, y);
            if (b != -1) {
                if (ts[0].l.x1 == p.r.x1) {
                    return make_pair(p.r.x1, y);
                } else {
                    return make_pair(p.r.x2, y);
                }
            }
        }        
    }

    if (layers[l].xsame) {
        int x = (p.r.x1 + p.r.x2) / 2;
        for (auto &ts : dtracks[l]) {
            int b = binary_search_x(ts, x);
            if (b != -1) {
                return make_pair(x, p.r.y1);
            }
        }
    } else {
        int y = (p.r.y1 + p.r.y2) / 2;
        for (auto &ts : dtracks[l]) {
            int b = binary_search_y(ts, y);
            if (b != -1) {
                return make_pair(p.r.x2, y);
            }
        }        
    }
    assert(1 == 0);
}

pair<int,int> findPathxy (pin p) {
    // cout << p.r.x1 << " " << p.r.x2 << " " << p.r.y1 << " " << p.r.y2 << endl;
    int l = p.layername;
    if (layers[l].xsame) {
        int x = (p.r.x1 + p.r.x2) / 2;
        for (auto &ts : dtracks[l]) {
            if (ts[0].l.y1 != p.r.y1 && ts[0].l.y2 != p.r.y2) {
                continue;
            }
            int b = binary_search_x(ts, x);
            if (b != -1) {
                if (ts[0].l.y1 == p.r.y1) {
                    return make_pair(x, p.r.y2);
                } else {
                    return make_pair(x, p.r.y1);
                }
            }
        }
    } else {
        int y = (p.r.y1 + p.r.y2) / 2;
        for (auto &ts : dtracks[l]) {
            if (ts[0].l.x1 != p.r.x1 && ts[0].l.x2 != p.r.x2) {
                continue;
            }
            int b = binary_search_y(ts, y);
            if (b != -1) {
                if (ts[0].l.x1 == p.r.x1) {
                    return make_pair(p.r.x2, y);
                } else {
                    return make_pair(p.r.x1, y);
                }
            }
        }        
    }

    if (layers[l].xsame) {
        int x = (p.r.x1 + p.r.x2) / 2;
        for (auto &ts : dtracks[l]) {
            int b = binary_search_x(ts, x);
            if (b != -1) {
                return make_pair(x, p.r.y2);
            }
        }
    } else {
        int y = (p.r.y1 + p.r.y2) / 2;
        for (auto &ts : dtracks[l]) {
            int b = binary_search_y(ts, y);
            if (b != -1) {
                return make_pair(p.r.x1, y);
            }
        }        
    }
    assert(1 == 0);
}

vector<direction> topologyAnalyses(vector<pair<int,pair<int, int> > > v) {
    vector<direction> ret;
    int s = v.size();
    for (int i = 0; i < s -1; ++i) {
        direction cur;
        if (v[i+1].first > v[i].first) {
            cur = front;
        } else if (v[i+1].first < v[i].first) {
            cur = back;
        } else if (v[i+1].second.first > v[i].second.first) {
            cur = ri;
        } else if (v[i+1].second.first < v[i].second.first) {
            cur = le;
        } else if (v[i+1].second.second > v[i].second.second) {
            cur = up;
        } else if (v[i+1].second.second < v[i].second.second) {
            cur = down;
        } else {
            assert(false);
        }
        ret.push_back(cur);
    }
    return ret;
}

map<vector<direction>, int> topologySet;

pair<int, int> getTrackIntersection(const tuple<int, int, int> t1, const tuple<int, int, int> t2) {
    if (get<0>(t1) != get<0>(t2)) {
        return getCross(dtracks[get<0>(t1)][get<1>(t1)][get<2>(t1)].l, dtracks[get<0>(t2)][get<1>(t2)][get<2>(t2)].l);
    } else {
        auto &l1 = dtracks[get<0>(t1)][get<1>(t1)][get<2>(t1)].l;
        auto &l2 = dtracks[get<0>(t2)][get<1>(t2)][get<2>(t2)].l;
        if (l1.x1 == l2.x2 && l1.y1 == l2.y2) return make_pair(l1.x1, l1.y1);
        if (l1.x2 == l2.x1 && l1.y2 == l2.y1) return make_pair(l1.x2, l1.y2);
        assert(1 == 0);
    }
}

void add_use(vector<pair<int, pair<int, int>>> path, vector<tuple<int, int, int>> patht) {
    int s = path.size();
    int c1 = 0;
    int c2 = 0;
    while(c1 < s -1) {
        if (path[c1].first == path[c1+1].first) {
            auto &t1 = patht[c2];
            // cout << path[c1].second.first << " " << path[c1].second.second << " " << path[c1+1].second.first << " " << path[c1+1].second.second << endl;
            layers[dtracks[get<0>(t1)][get<1>(t1)][get<2>(t1)].layername].add_temp(path[c1].second.first, path[c1].second.second, path[c1+1].second.first,path[c1+1].second.second);
            ++c2;
        }
        ++c1;
    }
}

// void add_use(vector<vector<pair<int, pair<int, int>>>> path, vector<vector<tuple<int, int, int>>> patht) {
//     int s = patht.size();
//     for (int c = 0; c < s; ++c) {
//         add_use(path[c], patht[c]);
//     }
// }

vector<vector<pair<int, pair<int, int>>>> v2vv(vector<pair<int, pair<int, int>>> v) {
    vector<vector<pair<int, pair<int, int>>>> ret;
    vector<pair<int, pair<int, int>>> c;
    int last = -1;
    int s = v.size();
    for (int i = 0; i < s; ++i) {
        if (i == 0) {
            last = v[i].first;
        } else if (last != v[i].first) {
            ret.push_back(c);
            c.clear();
            last = v[i].first;
        }
        c.push_back(v[i]);
    }
    ret.push_back(c);
    return ret;
}

vector<vector<tuple<int,int,int>>> v2vv(vector<tuple<int,int,int>> v) {
    vector<vector<tuple<int,int,int>>> ret;
    vector<tuple<int,int,int>> c;
    int last = -1;
    int s = v.size();
    for (int i = 0; i < s; ++i) {
        if (i == 0) {
            last = get<0>(v[i]);
        } else if (last != get<0>(v[i])) {
            ret.push_back(c);
            c.clear();
            last = get<0>(v[i]);
        }
        c.push_back(v[i]);
    }
    ret.push_back(c);
    return ret;
}

template <typename T>
vector<T> vv2v(vector<vector<T>> v) {
    vector<pair<int, pair<int, int>>> c;
    for (auto i : v) {
        for (auto j : i) {
            c.push_back(j);
        }
    }
    return c;
}

vector<pair<int, pair<int, int>>> pin0_path;
vector<tuple<int, int, int>> pin0_patht;
vector<pair<int, pair<int, int>>> pin1_path;
vector<tuple<int, int, int>> pin1_patht;

int generateSinglePath(newBus nb, int ii = 0, bool greverse = false) {
    pin0_path.clear();
    pin0_patht.clear();
    pin1_path.clear();
    pin1_patht.clear();
    int snb = nb.pins.size();
    // cout << "snb: " << snb << endl;
    if (ii == -1) {
        ii = snb - 1;
    }
    int startii = ii;
    auto p0 = findPinxy(nb.pins[ii][0]);
    auto p1 = findPinxy(nb.pins[ii][1]);
    grid g(nb.pins[ii][0].layername, p0.first, p0.second,nb.pins[ii][1].layername, p1.first, p1.second,0);
    auto vv = astar(g);
    // cout << "astar" << endl;
    vector<tuple<int, int, int>> vtrack;
    vector<vector<tuple<int, int, int>>> dvtrack;
    int lastl = -1;
    for (auto pp : vv) {
        auto tuplet = findTrackIdx(pp.first, pp.second.first, pp.second.second);
        // cout << pp.first << " " << pp.second.first << " " << pp.second.second << endl;
        // cout << get<0>(tuplet) << " " << get<1>(tuplet) << " " << get<2>(tuplet) << endl;
        if (get<0>(tuplet) != lastl) {
            if (lastl != -1) {
                dvtrack.push_back(vtrack);
            }
            vtrack.clear();
            lastl = get<0>(tuplet);
        }
        vtrack.push_back(tuplet);
    }
    // cout << "********************************" << endl << endl;
    dvtrack.push_back(vtrack);
    vtrack.clear();

    int sd = dvtrack.size();
    vector<bool> addj(sd);
    int cl = 0;
    int &cj = cl;
    int x;
    int y;

    for (int i = 0; i < sd; ++i) {
        int li = dvtrack[i].size();
        if (li == 1) {
            if (i == sd - 1) {
                pin0_path.push_back(make_pair(vv[cj].first, make_pair(p1.first, p1.second)));
            } else {
                pin0_path.push_back(make_pair(vv[cj-1].first, make_pair(x, y)));
            }
        } else if (i == 0) {
            for (int j = 0; j < li-1; ++j) {
                pin0_path.push_back(vv[cl + j]);
                if (j == 0 || dvtrack[i][j -1] != dvtrack[i][j]) {
                    pin0_patht.push_back(dvtrack[i][j]);
                }
                else pin0_patht.push_back(dvtrack[i][j+1]);
            }
        } else if (i == sd - 1 || i == sd - 2 && dvtrack[i+1].size() == 1) {
            int cur_layer = vv[cl].first;
            x = pin0_path[cl - 2].second.first;
            y = pin0_path[cl - 2].second.second;
            if (layers[cur_layer].xsame) {
                x = vv[cl].second.first;
                pin0_path.push_back(make_pair(vv[cj -1].first, make_pair(x, y)));
                for (int j = 0; j < li; ++j) {
                    if (j == 0)
                        pin0_path.push_back(make_pair(vv[cj+j].first, make_pair(x, y)));
                    else
                        pin0_path.push_back(make_pair(vv[cj+j].first, make_pair(x, vv[cj + j].second.second)));
                    if (j != li - 1) {
                        if (j == 0 || dvtrack[i][j -1] != dvtrack[i][j]) {
                            pin0_patht.push_back(dvtrack[i][j]);
                        } else pin0_patht.push_back(dvtrack[i][j+1]);
                    }

                }
            } else {
                y = vv[cl].second.second;
                pin0_path.push_back(make_pair(vv[cj -1].first, make_pair(x, y)));
                for (int j = 0; j < li; ++j) {
                    if (j == 0)
                        pin0_path.push_back(make_pair(vv[cj+j].first, make_pair(x, y)));
                    else
                        pin0_path.push_back(make_pair(vv[cj+j].first, make_pair(vv[cj + j].second.first, y)));
                    if (j != li - 1) {
                        if (j == 0 || dvtrack[i][j -1] != dvtrack[i][j]) {
                            pin0_patht.push_back(dvtrack[i][j]);
                        } else pin0_patht.push_back(dvtrack[i][j+1]);
                    }
                }
            }
        } else {
            int cur_layer = vv[cl].first;
            x = pin0_path[cl - 2].second.first;
            y = pin0_path[cl - 2].second.second;
            if (layers[cur_layer].xsame) {
                bool found = false;
                int mid_j = get<2>(dvtrack[i][0]);
                int min_j = 0;
                int max_j = dtracks[cur_layer][get<1>(dvtrack[i][0])].size() - 1;
                int cur_j = min_j;
                while(!found && cur_j < max_j) {
                    x = dtracks[cur_layer][get<1>(dvtrack[i][0])][cur_j].l.x1;
                    bool fail = false;
                    for (int j = 0; j < li - 1; ++j) {
                        if (j == 0 || dvtrack[i][j -1] != dvtrack[i][j]) {
                            auto &tv = dtracks[cur_layer][get<1>(dvtrack[i][j])];
                            auto b = binary_search_x(tv, x);
                            if (b == -1 || !layers[tv[b].layername].can_use(x, vv[cl+j].second.second, x, vv[cl+j+1].second.second)) {
                                fail = true;
                                break;
                            }
                        } else {
                            auto &tv = dtracks[cur_layer][get<1>(dvtrack[i][j+1])];
                            auto b = binary_search_x(tv, x);
                            if (b == -1 || !layers[tv[b].layername].can_use(x, vv[cl+j].second.second, x, vv[cl+j+1].second.second)) {
                                fail = true;
                                break;
                            }
                        }        
                    }
                    if (fail) {
                        ++cur_j;
                        continue;
                    }
                    found = true;
                    addj[i] = true;
                    pin0_path.push_back(make_pair(vv[cj - 1].first, make_pair(x, y)));
                    for (int j = 0; j < li - 1; ++j) {
                        if (j > 0)
                            pin0_path.push_back(make_pair(vv[cj + j].first, make_pair(x, vv[cj + j].second.second)));
                        else
                            pin0_path.push_back(make_pair(vv[cj + j].first, make_pair(x, y)));
                        if (j == 0 || dvtrack[i][j -1] != dvtrack[i][j]) {
                            auto &tv = dtracks[cur_layer][get<1>(dvtrack[i][j])];
                            auto b = binary_search_x(tv, x);
                            pin0_patht.push_back(make_tuple(get<0>(dvtrack[i][j]), get<1>(dvtrack[i][j]), b));
                        } else {
                            auto &tv = dtracks[cur_layer][get<1>(dvtrack[i][j+1])];
                            auto b = binary_search_x(tv, x);
                            pin0_patht.push_back(make_tuple(get<0>(dvtrack[i][j+1]), get<1>(dvtrack[i][j+1]), b));
                        }        
                    }
                    y = vv[cj + li - 1].second.second;
                }
                assert(found);
                if (!found){
                    cur_j = mid_j;
                    while(!found && cur_j > min_j) {
                        x = dtracks[cur_layer][get<1>(dvtrack[i][0])][cur_j].l.x1;
                        bool fail = false;
                        for (int j = 0; j < li - 1; ++j) {
                            if (j == 0 || dvtrack[i][j -1] != dvtrack[i][j]) {
                                auto &tv = dtracks[cur_layer][get<1>(dvtrack[i][j])];
                                auto b = binary_search_x(tv, x);
                                if (b == -1 || !layers[tv[b].layername].can_use(x, vv[cl+j].second.second, x, vv[cl+j+1].second.second)) {
                                    fail = true;
                                    break;
                                }
                            } else {
                                auto &tv = dtracks[cur_layer][get<1>(dvtrack[i][j+1])];
                                auto b = binary_search_x(tv, x);
                                if (b == -1 || !layers[tv[b].layername].can_use(x, vv[cl+j].second.second, x, vv[cl+j+1].second.second)) {
                                    fail = true;
                                    break;
                                }
                            }        
                        }
                        if (fail) {
                            --cur_j;
                            continue;
                        }
                        found = true;
                        addj[i] = false;
                        pin0_path.push_back(make_pair(vv[cj - 1].first, make_pair(x, y)));
                        for (int j = 0; j < li - 1; ++j) {
                            if (j > 0)
                                pin0_path.push_back(make_pair(vv[cj + j].first, make_pair(x, vv[cj + j].second.second)));
                            else
                                pin0_path.push_back(make_pair(vv[cj + j].first, make_pair(x, y)));
                            if (j == 0 || dvtrack[i][j -1] != dvtrack[i][j]) {
                                auto &tv = dtracks[cur_layer][get<1>(dvtrack[i][j])];
                                auto b = binary_search_x(tv, x);
                                pin0_patht.push_back(make_tuple(get<0>(dvtrack[i][j]), get<1>(dvtrack[i][j]), b));
                            } else {
                                auto &tv = dtracks[cur_layer][get<1>(dvtrack[i][j+1])];
                                auto b = binary_search_x(tv, x);
                                pin0_patht.push_back(make_tuple(get<0>(dvtrack[i][j+1]), get<1>(dvtrack[i][j+1]), b));
                            }        
                        }
                        y = vv[cj + li - 1].second.second;
                    }
                }
                assert(found);
            } else {
                bool found = false;
                int mid_j = get<2>(dvtrack[i][0]);
                int min_j = 0;
                int max_j = dtracks[cur_layer][get<1>(dvtrack[i][0])].size() - 1;
                int cur_j = min_j;
                while(!found && cur_j < max_j) {
                    // cout << "cur_layer: " << cur_layer << "get<1>(dvtrack[i][0]):" << get<1>(dvtrack[i][0]) << "cur_j:" << cur_j << endl;
                    y = dtracks[cur_layer][get<1>(dvtrack[i][0])][cur_j].l.y1;
                    bool fail = false;
                    for (int j = 0; j < li - 1; ++j) {
                        if (j == 0 || dvtrack[i][j -1] != dvtrack[i][j]) {
                            auto &tv = dtracks[cur_layer][get<1>(dvtrack[i][j])];
                            auto b = binary_search_y(tv, y);
                            if (b == -1 || !layers[tv[b].layername].can_use(vv[cl+j].second.first, y, vv[cl+j+1].second.first, y)) {
                                fail = true;
                                break;
                            }
                        } else {
                            auto &tv = dtracks[cur_layer][get<1>(dvtrack[i][j+1])];
                            auto b = binary_search_y(tv, y);
                            if (b == -1 || !layers[tv[b].layername].can_use(vv[cl+j].second.first, y, vv[cl+j+1].second.first, y)) {
                                fail = true;
                                break;
                            }
                        }        
                    }
                    if (fail) {
                        ++cur_j;
                        continue;
                    }
                    found = true;
                    addj[i] = true;
                    pin0_path.push_back(make_pair(vv[cj - 1].first, make_pair(x, y)));
                    for (int j = 0; j < li - 1; ++j) {
                        if (j > 0)
                            pin0_path.push_back(make_pair(vv[cj + j].first, make_pair(vv[cj + j].second.first, y)));
                        else
                            pin0_path.push_back(make_pair(vv[cj + j].first, make_pair(x, y)));
                        if (j == 0 || dvtrack[i][j -1] != dvtrack[i][j]) {
                            auto &tv = dtracks[cur_layer][get<1>(dvtrack[i][j])];
                            auto b = binary_search_y(tv, y);
                            pin0_patht.push_back(make_tuple(get<0>(dvtrack[i][j]), get<1>(dvtrack[i][j]), b));
                        } else {
                            auto &tv = dtracks[cur_layer][get<1>(dvtrack[i][j+1])];
                            auto b = binary_search_y(tv, y);
                            pin0_patht.push_back(make_tuple(get<0>(dvtrack[i][j+1]), get<1>(dvtrack[i][j+1]), b));
                        }        
                    }
                    x = vv[cj + li - 1].second.first;
                }
                assert(found);
                if (!found){
                    cur_j = mid_j;
                    while(!found && cur_j > min_j) {
                        // cout << "cur_layer: " << cur_layer << "get<1>(dvtrack[i][0]):" << get<1>(dvtrack[i][0]) << "cur_j:" << cur_j << endl;
                        y = dtracks[cur_layer][get<1>(dvtrack[i][0])][cur_j].l.y1;
                        bool fail = false;
                        for (int j = 0; j < li - 1; ++j) {
                            if (j == 0 || dvtrack[i][j -1] != dvtrack[i][j]) {
                                auto &tv = dtracks[cur_layer][get<1>(dvtrack[i][j])];
                                auto b = binary_search_y(tv, y);
                                if (b == -1 || !layers[tv[b].layername].can_use(vv[cl+j].second.first, y, vv[cl+j+1].second.first, y)) {
                                    fail = true;
                                    break;
                                }
                            } else {
                                auto &tv = dtracks[cur_layer][get<1>(dvtrack[i][j+1])];
                                auto b = binary_search_y(tv, y);
                                if (b == -1 || !layers[tv[b].layername].can_use(vv[cl+j].second.first, y, vv[cl+j+1].second.first, y)) {
                                    fail = true;
                                    break;
                                }
                            }        
                        }
                        if (fail) {
                            --cur_j;
                            continue;
                        }
                        found = true;
                        addj[i] = false;
                        pin0_path.push_back(make_pair(vv[cj - 1].first, make_pair(x, y)));
                        for (int j = 0; j < li - 1; ++j) {
                            if (j > 0)
                                pin0_path.push_back(make_pair(vv[cj + j].first, make_pair(vv[cj + j].second.first, y)));
                            else
                                pin0_path.push_back(make_pair(vv[cj + j].first, make_pair(x, y)));
                            if (j == 0 || dvtrack[i][j -1] != dvtrack[i][j]) {
                                auto &tv = dtracks[cur_layer][get<1>(dvtrack[i][j])];
                                auto b = binary_search_y(tv, y);
                                pin0_patht.push_back(make_tuple(get<0>(dvtrack[i][j]), get<1>(dvtrack[i][j]), b));
                            } else {
                                auto &tv = dtracks[cur_layer][get<1>(dvtrack[i][j+1])];
                                auto b = binary_search_y(tv, y);
                                pin0_patht.push_back(make_tuple(get<0>(dvtrack[i][j+1]), get<1>(dvtrack[i][j+1]), b));
                            }        
                        }
                        x = vv[cj + li - 1].second.first;
                    }
                }
                assert(found);

            }

        }
        cl += li;
    }
    final_path[vv[0]] = pin0_path;
    
    // for (auto ii : pin0_path) {
    //     cout << ii.first << " " << ii.second.first << " " << ii.second.second << endl;
    // }
    // for (auto ii : pin0_patht) {
    //     cout << get<0>(ii) << " " << get<1>(ii) << " " << get<2>(ii) << endl;
    // }
    ii = 0;
    if (greverse) {
        ii = snb - 1;
    }
    while(ii != -1 && ii != snb){
        auto xy1 = findPinxy(nb.pins[ii][0]);
        auto xy2 = findPinxy(nb.pins[ii][1]);
        auto vvpp0 = v2vv(pin0_path);
        auto vvt0 = v2vv(pin0_patht);
        cl = 0;
        int x = xy1.first;
        int y = xy1.second;
        int vti = 0;
        int lastx = x;
        int lasty = y;
        int lastlayer = -1;
        
        for (int i = 0; i < sd; ++i, ++vti) {
            auto& cur_vpp = vvpp0[i];
            int li = cur_vpp.size();
            if (li == 1) {
                if (i == sd - 1) {
                    pin1_path.push_back(make_pair(vvpp0[i][0].first, make_pair(xy2.first, xy2.second)));
                } else {
                    pin1_path.push_back(make_pair(vvpp0[i-1][0].first, make_pair(x, y)));
                }
                --vti;
                continue;
            }
            auto& cur_vt = vvt0[vti];
            if (i > 0) {
                lastlayer = get<0>(vvt0[vti - 1][0]);
            }
            int cur_layer = cur_vpp[0].first;
            if (i == 0) {
                if (layers[cur_layer].xsame) {
                    for (int j = 0; j < li - 1; ++j) {
                        pin1_path.push_back(make_pair(cur_layer, make_pair(x, cur_vpp[j].second.second)));
                        auto b = binary_search_x(dtracks[cur_layer][get<1>(cur_vt[j])], x);
                        pin1_patht.push_back(make_tuple(cur_layer,get<1>(cur_vt[j]),b));
                    }
                    lasty = y = cur_vpp[li-1].second.second;
                } else {
                    for (int j = 0; j < li-1; ++j) {
                        pin1_path.push_back(make_pair(cur_layer, make_pair(cur_vpp[j].second.first, y)));
                        auto b = binary_search_y(dtracks[cur_layer][get<1>(cur_vt[j])], y);
                        pin1_patht.push_back(make_tuple(cur_layer,get<1>(cur_vt[j]),b));
                    }
                    lastx = x = cur_vpp[li-1].second.first;                    
                }
            } else if (i == sd - 1 || i == sd - 2 && vvpp0[i+1].size() == 1) {
                if (layers[cur_layer].xsame) {
                    x = xy2.first;
                    pin1_path.push_back(make_pair(vvpp0[i-1][0].first, make_pair(x, y)));
                    // if (!layers[lastlayer].can_use(lastx, lasty, x, y)) {
                    //     return ii;
                    // } 
                    for (int j = 0; j < li - 1; ++j) {
                        if (j == 0) {
                            pin1_path.push_back(make_pair(cur_layer, make_pair(x, y)));
                        } else {
                            pin1_path.push_back(make_pair(cur_layer, make_pair(x, cur_vpp[j].second.second)));
                        }
                        auto b = binary_search_x(dtracks[cur_layer][get<1>(cur_vt[j])], x);
                        int jk = get<1>(cur_vt[j]);
                        if (b == -1) {
                            return ii;
                            // int maxj = dvtrack[cur_layer].size();
                            // for (jk = 0; jk < maxj; ++jk) {
                            //     b = binary_search_x(dtracks[cur_layer][jk], x);
                            //     if (b!=-1) {
                            //         break;
                            //     }
                            // }
                        }
                        if (!layers[cur_layer].can_use( x, cur_vpp[j].second.second, x, cur_vpp[j+1].second.second )) {
                            return ii;
                        }
                        pin1_patht.push_back(make_tuple(cur_layer, jk ,b));
                    }
                    y = xy2.second;
                    pin1_path.push_back(make_pair(vvpp0[i][0].first, make_pair(x, y)));
                } else {
                    y = xy2.second;
                    pin1_path.push_back(make_pair(vvpp0[i-1][0].first, make_pair(x, y)));
                    // if (!layers[lastlayer].can_use(lastx, lasty, x, y)) {
                    //     return ii;
                    // } 
                    for (int j = 0; j < li-1; ++j) {
                        if (j == 0) {
                            pin1_path.push_back(make_pair(cur_layer, make_pair(x, y)));
                        } else {
                            pin1_path.push_back(make_pair(cur_layer, make_pair(cur_vpp[j].second.first, y)));
                        }
                        auto b = binary_search_y(dtracks[cur_layer][get<1>(cur_vt[j])], y);
                        int jk = get<1>(cur_vt[j]);
                        if (b == -1) {
                            return ii;
                            // int maxj = dvtrack[cur_layer].size();
                            // for (jk = 0; jk < maxj; ++jk) {
                            //     b = binary_search_y(dtracks[cur_layer][jk], y);
                            //     if (b!=-1) {
                            //         break;
                            //     }
                            // }
                        }
                        if (!layers[cur_layer].can_use( cur_vpp[j].second.first, y, cur_vpp[j+1].second.first  ,y )) {
                            return ii;
                        }
                        pin1_patht.push_back(make_tuple(cur_layer, jk ,b));
                    }
                    x = xy2.first;     
                    pin1_path.push_back(make_pair(vvpp0[i][0].first, make_pair(x, y)));               
                }
            } else {
                bool found = false;
                int mid_j = get<2>(cur_vt[0]);
                int min_j = -1;
                int max_j = dtracks[cur_layer][get<1>(cur_vt[0])].size();
                int cur_j = mid_j;
                if (layers[cur_layer].xsame) {
                    if (addj[i]) {
                        if (ii != 0)
                            ++cur_j;
                        while(!found && cur_j < max_j) {
                            if (i != sd - 3 || vvpp0[i+1].size() != 1 ) {
                                x = dtracks[cur_layer][get<1>(cur_vt[0])][cur_j].l.x1;
                                bool fail = false;
                                for (int j = 0; j < li - 1; ++j) {
                                    auto &tv = dtracks[cur_layer][get<1>(cur_vt[j])];
                                    auto b = binary_search_x(tv, x);
                                    if (b == -1 || !layers[tv[b].layername].can_use(x, cur_vpp[j].second.second, x, cur_vpp[j+1].second.second)) {
                                        fail = true;
                                        break;
                                    }
                                }
                                if (fail) {
                                    ++cur_j;
                                    continue;
                                }
                            }else { x = xy2.first;}
                            found = true;
                            // cout << "ii:" << ii << ",cur_j:" << cur_j << endl;
                            // if (!layers[lastlayer].can_use(lastx, lasty, x, y)) {
                            //     return ii;
                            // } 
                            pin1_path.push_back(make_pair(vvpp0[i-1][0].first, make_pair(x, y)));
                            for (int j = 0; j < li - 1; ++j) {
                                if (j == 0) {
                                    pin1_path.push_back(make_pair(cur_layer, make_pair(x, y)));
                                } else {
                                    pin1_path.push_back(make_pair(cur_layer, make_pair(x, cur_vpp[j].second.second)));
                                }
                                auto &tv = dtracks[cur_layer][get<1>(cur_vt[j])];
                                auto b = binary_search_x(tv, x);
                                if (b==-1) {return ii;}
                                // if (!layers[cur_layer].can_use( x, cur_vpp[j].second.second, x, cur_vpp[j+1].second.second )) {
                                //     return ii;
                                // }
                                pin1_patht.push_back(make_tuple(cur_layer,get<1>(cur_vt[j]),b));
                            }
                        }
                    } else {
                        if (ii != 0)
                            --cur_j;
                        while(!found && cur_j > min_j) {
                            if (i != sd - 3 || vvpp0[i+1].size() != 1 ) {
                                x = dtracks[cur_layer][get<1>(cur_vt[0])][cur_j].l.x1;
                                bool fail = false;
                                for (int j = 0; j < li - 1; ++j) {
                                    auto &tv = dtracks[cur_layer][get<1>(cur_vt[j])];
                                    auto b = binary_search_x(tv, x);
                                    if (b == -1 || !layers[tv[b].layername].can_use(x, cur_vpp[j].second.second, x, cur_vpp[j+1].second.second)) {
                                        fail = true;
                                        break;
                                    }
                                }
                                if (fail) {
                                    --cur_j;
                                    continue;
                                }
                            }else { x = xy2.first;}
                            found = true;
                            // if (!layers[lastlayer].can_use(lastx, lasty, x, y)) {
                            //     return ii;
                            // } 
                            pin1_path.push_back(make_pair(vvpp0[i-1][0].first, make_pair(x, y)));
                            for (int j = 0; j < li - 1; ++j) {
                                if (j == 0) {
                                    pin1_path.push_back(make_pair(cur_layer, make_pair(x, y)));
                                } else {
                                    pin1_path.push_back(make_pair(cur_layer, make_pair(x, cur_vpp[j].second.second)));
                                }
                                auto &tv = dtracks[cur_layer][get<1>(cur_vt[j])];
                                auto b = binary_search_x(tv, x);
                                if (b==-1) {return ii;}
                                pin1_patht.push_back(make_tuple(cur_layer,get<1>(cur_vt[j]),b));
                            }
                        }
                    }
                    y = cur_vpp[li-1].second.second;
                    lastx = x;
                    lasty = cur_vpp[max(li-2, 0)].second.second;
                } else {
                    if (addj[i]) {
                        if (ii != 0)
                            ++cur_j;
                        while(!found && cur_j < max_j) {
                            if (i != sd - 3 || vvpp0[i+1].size() != 1 ) {
                                y = dtracks[cur_layer][get<1>(cur_vt[0])][cur_j].l.y1;
                                bool fail = false;
                                for (int j = 0; j < li - 1; ++j) {
                                    auto &tv = dtracks[cur_layer][get<1>(cur_vt[j])];
                                    auto b = binary_search_y(tv, y);
                                    if (b == -1 || !layers[tv[b].layername].can_use(cur_vpp[j].second.first, y, cur_vpp[j+1].second.first, y)) {
                                        fail = true;
                                        break;
                                    }
                                }
                                if (fail) {
                                    ++cur_j;
                                    continue;
                                }
                            }else { y = xy2.second;}
                            found = true;
                            // if (!layers[lastlayer].can_use(lastx, lasty, x, y)) {
                            //     return ii;
                            // } 
                            pin1_path.push_back(make_pair(vvpp0[i-1][0].first, make_pair(x, y)));
                            for (int j = 0; j < li - 1; ++j) {
                                if (j == 0) {
                                    pin1_path.push_back(make_pair(cur_layer, make_pair(x, y)));
                                } else {
                                    pin1_path.push_back(make_pair(cur_layer, make_pair(cur_vpp[j].second.first, y)));
                                }
                                auto &tv = dtracks[cur_layer][get<1>(cur_vt[j])];
                                auto b = binary_search_y(tv, y);
                                if (b==-1) {return ii;}
                                pin1_patht.push_back(make_tuple(cur_layer,get<1>(cur_vt[j]),b));
                            }
                        }
                    } else {
                        if (ii != 0)
                            --cur_j;
                        while(!found && cur_j > min_j) {
                            if (i != sd - 3 || vvpp0[i+1].size() != 1 ) {
                                y = dtracks[cur_layer][get<1>(cur_vt[0])][cur_j].l.y1;
                                bool fail = false;
                                for (int j = 0; j < li - 1; ++j) {
                                    auto &tv = dtracks[cur_layer][get<1>(cur_vt[j])];
                                    auto b = binary_search_y(tv, y);
                                    if (b == -1 || !layers[tv[b].layername].can_use(cur_vpp[j].second.first, y, cur_vpp[j+1].second.first, y)) {
                                        fail = true;
                                        break;
                                    }
                                }
                                if (fail) {
                                    --cur_j;
                                    continue;
                                }
                            }else { y = xy2.second;}
                            found = true;
                            // if (!layers[lastlayer].can_use(lastx, lasty, x, y)) {
                            //     return ii;
                            // } 
                            pin1_path.push_back(make_pair(vvpp0[i-1][0].first, make_pair(x, y)));
                            for (int j = 0; j < li - 1; ++j) {
                                if (j == 0) {
                                    pin1_path.push_back(make_pair(cur_layer, make_pair(x, y)));
                                } else {
                                    pin1_path.push_back(make_pair(cur_layer, make_pair(cur_vpp[j].second.first, y)));
                                }
                                auto &tv = dtracks[cur_layer][get<1>(cur_vt[j])];
                                auto b = binary_search_y(tv, y);
                                if (b==-1) {return ii;}
                                pin1_patht.push_back(make_tuple(cur_layer,get<1>(cur_vt[j]),b));
                            }
                        }
                    }
                    
                    x = cur_vpp[li-1].second.first;
                    lastx = cur_vpp[max(li-2, 0)].second.first;
                    lasty = y;
                }
                // assert(found);
                if (!found) return ii;
                assert(found);
            }
            cl += li;
        }
        final_path[make_pair(nb.pins[ii][0].layername,xy1)] = pin1_path;
        // if (startii != 0) {
        // for (auto iii: pin1_path) {
        //     cout << iii.first << " " << iii.second.first << " " << iii.second.second << endl;
        // }
        // for (auto iii: pin1_patht) {
        //     cout << get<0>(iii) << " " << get<1>(iii) << " " << get<2>(iii) << endl;
        // } 
        // }
        add_use(pin1_path, pin1_patht);
        // addl.push_back(pin1_path);
        // addtl.push_back(pin1_patht);
        pin0_path = pin1_path;
        pin0_patht = pin1_patht;
        pin1_path.clear();
        pin1_patht.clear();
        ++ii;
        if (greverse) ii -=2;
    }
    return -1;
}

void cleartemp(int c) {
    int l = layers.size();
    for (int i = 0; i < l; ++i) {
        layers[i].clear_temp(c);
    }
}

bool generateGuides() {
    int c = 0;
    while (!newBuses.empty()) {
        // cout << ++c << endl;
        auto nb = newBuses.top();
        newBuses.pop();
        // int k = 1;
        // vector<int> v(1, 0);
        int k = min(nb.nb, max(10,min(50,nb.nb/4)));
        vector<int> v = randomk(k, nb.pins.size());
        topologySet.clear();
        
        // for (auto i: v) {
        //     auto p0 = findPinxy(nb.pins[i][0]);
        //     auto p1 = findPinxy(nb.pins[i][1]);
        //     grid g(nb.pins[i][0].layername, p0.first, p0.second,nb.pins[i][1].layername, p1.first, p1.second,0);
        //     // cout << p0.first << " " << p0.second << endl;
        //     // cout << p1.first << " " << p1.second << endl;
        //     auto vv = astar(g);
        //     // for (auto ii: vv) {
        //     //     cout << ii.first << ' ' << ii.second.first << ' ' << ii.second.second << endl;
        //     // }
        //     // cout << endl << endl;
        //     auto ta = topologyAnalyses(vv);
        //     // for (auto ii : ta) {
        //     //     cout << ii << ' ';
        //     // }
        //     // cout << endl;
        //     auto it = topologySet.find(ta);
        //     if (it == topologySet.end()) {
        //         topologySet[ta] = 1;
        //     } else {
        //         ++topologySet[ta];
        //     }
        // }
        // for (auto it = topologySet.begin(); it != topologySet.end(); ++it) {
        //     printv(it->first);
        //     cout << it->second << endl;
        // }
        // cout << endl;
        // cout << c << endl;
        // cout << "c:" << c << endl;
        // cout << "nb.pins.size()" << nb.pins.size() << endl;
        int success = generateSinglePath(nb);
        int ret;
        
        // cout << "success:" << success << endl;
        cleartemp(success);
        while (success != -1) {
            ret = generateSinglePath(nb, success);
            // cout << "ret:" << ret << endl;
            if (ret != -1 && ret <= success) {
                break;
            }
            success = ret;
            cleartemp(success);
        }
        if (success == -1) {
            continue;
        }
        ret = -1;
        success = nb.pins.size() - 1;
        while (success != -1) {
            ret = generateSinglePath(nb, success, true);
            // cout << "ret:" << ret << endl;
            if (ret != -1 && ret >= success) {
                return false;
            }
            success = ret;
            cleartemp(success);
        }
    }
    return true;
}

void printPath(){
    int sb = buses.size();
    for (int i = 0; i < sb; ++i) {
        printf("BUS %s\n", buses[i].name.c_str());
        int l = buses[i].pins.size();
        for (int j = 0; j < l; ++j) {
            printf("BIT %s\n", buses[i].pins[j][0].name.c_str());
            auto xy = findPinxy(buses[i].pins[j][0]);
            auto layername = buses[i].pins[j][0].layername;
            auto key = make_pair(layername, xy);
            auto v = final_path[key];
            int s = v.size();
            --s;
            printf("PATH %d\n", s);
            v[0].second = findPathxy(buses[i].pins[j][0]);
            v[s].second = findPathxy(buses[i].pins[j][1]);
            if (!buses[i].pins[j][0].reverse) {
                for (int k = 0; k < s; ++k) {
                    if (v[k].first == v[k+1].first) {
                        int minx = min(v[k].second.first, v[k+1].second.first);
                        int maxx = max(v[k].second.first, v[k+1].second.first);
                        int miny = min(v[k].second.second, v[k+1].second.second);
                        int maxy = max(v[k].second.second, v[k+1].second.second);
                        printf("%s (%d %d) (%d %d)\n",layers[v[k].first].name.c_str(), minx, miny, maxx, maxy);
                    } else {
                        int minl = min(v[k].first, v[k+1].first);
                        printf("%s (%d %d)\n",layers[minl].name.c_str(), v[k].second.first, v[k].second.second);
                    }
                }
            } else {
                for (int k = s - 1; k >= 0; --k) {
                    if (v[k].first == v[k+1].first) {
                        int minx = min(v[k].second.first, v[k+1].second.first);
                        int maxx = max(v[k].second.first, v[k+1].second.first);
                        int miny = min(v[k].second.second, v[k+1].second.second);
                        int maxy = max(v[k].second.second, v[k+1].second.second);
                        printf("%s (%d %d) (%d %d)\n",layers[v[k].first].name.c_str(), minx, miny, maxx, maxy);
                    } else {
                        int minl = min(v[k].first, v[k+1].first);
                        printf("%s (%d %d)\n",layers[minl].name.c_str(), v[k].second.first, v[k].second.second);
                    }
                }
            }

            printf("ENDPATH\n");
            printf("ENDBIT\n");
        }
        printf("ENDBUS\n");
    }
}

int main(int argc, char** argv) {
    // cout << argc << endl;
    // for (auto i= 0; i < argc; ++i) {
    //     cout << argv[i] << endl;
    // }
    freopen(argv[1], "r", stdin);
    freopen(argv[2], "w", stdout);

    readInput();
    getSuperTrack();

    int cnt = 0;
    // cout << tracks.size() << endl;
    // cout << dtracks.size() << endl;
    // for (auto aaa : dtracks) {
    //     cout << aaa.size() << endl;
    //     for (auto aa : aaa) {
    //         cout << aa.size() << ' ';
    //         cout << aa[0].l.x1 << ' ' << aa[0].l.y1 << endl;
    //         cout << aa[0].l.x2 << ' ' << aa[0].l.y2 << endl;
    //         cnt += aa.size();
    //     }
    //     cout << endl;
    // }
    // cout << endl;

    getSuperBus();
    getSuperBusPins3();
    getSBus();

    getnewBuses();
    // while(!newBuses.empty()) {
    //     auto t = newBuses.top();
    //     newBuses.pop();
    //     cout << t.cost << endl;
    // }
    bool b = generateGuides();
    // cout << "b:" << b << endl;
    assert(b);
    printPath();
}