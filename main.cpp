#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <queue>
#include <iostream>
#include <cstring>
#include <algorithm>

#include "bus.h"
#include "coordinates.h"
#include "designBoundary.h"
#include "layer.h"
#include "obstacle.h"
#include "track.h"
#include "assert.h"

using namespace std;

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

void printv(vector<int> v) {
    int s = v.size();
    for (int i = 0; i < s; i++) {
        cout << v[i] << ' ';
    }
    cout << endl;
}

void printvp(vector<pair<int, int> >vp) {
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
vector<bus> buses;
vector<obstacle> obstacles;

vector<vector<int> > superBus;
vector<vector<vector<pair<int, int> > > > superBusPins;

vector<vector<int> > sBus;



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

    cin >> s >> nlayers;
    for (int i = 0; i < nlayers; ++i) {
        cin >> ss[0] >> ss[1] >> m;
        NameToLayer[ss[0]] = i;
        layers.push_back(layer(i, ss[0], ss[1], m));
    }
    cin >> s;

    cin >> s >> ntracks;
    for (int i = 0; i < ntracks; ++i) {
        cin >> ss[0] >> ss[1] >> ss[2] >> ss[3] >> ss[4] >> m;
        tracks.push_back(track(NameToLayer[ss[0]], mystoi(ss[1]), mystoi(ss[2]), mystoi(ss[3]), mystoi(ss[4]), m));
    }
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
            cin >> s >> s;
            {
                vector<pin> pins;
                cin >> ss[0] >> ss[1] >> ss[2] >> ss[3] >> ss[4];
                cin >> sss[0] >> sss[1] >> sss[2] >> sss[3] >> sss[4];
                if (NameToLayer[ss[0]] < NameToLayer[sss[0]]) {
                    pins.push_back(pin(NameToLayer[ss[0]], mystoi(ss[1]), mystoi(ss[2]), mystoi(ss[3]), mystoi(ss[4])));
                    pins.push_back(pin(NameToLayer[sss[0]], mystoi(sss[1]), mystoi(sss[2]), mystoi(sss[3]), mystoi(sss[4])));
                } else if (NameToLayer[ss[0]] > NameToLayer[sss[0]]) {
                    pins.push_back(pin(NameToLayer[sss[0]], mystoi(sss[1]), mystoi(sss[2]), mystoi(sss[3]), mystoi(sss[4])));
                    pins.push_back(pin(NameToLayer[ss[0]], mystoi(ss[1]), mystoi(ss[2]), mystoi(ss[3]), mystoi(ss[4])));
                } else if (mystoi(ss[1]) < mystoi(sss[1]) || (mystoi(ss[1]) == mystoi(sss[1]) && mystoi(ss[2]) < mystoi(sss[2]))) {
                    pins.push_back(pin(NameToLayer[ss[0]], mystoi(ss[1]), mystoi(ss[2]), mystoi(ss[3]), mystoi(ss[4])));
                    pins.push_back(pin(NameToLayer[sss[0]], mystoi(sss[1]), mystoi(sss[2]), mystoi(sss[3]), mystoi(sss[4])));                    
                } else {
                    pins.push_back(pin(NameToLayer[sss[0]], mystoi(sss[1]), mystoi(sss[2]), mystoi(sss[3]), mystoi(sss[4])));
                    pins.push_back(pin(NameToLayer[ss[0]], mystoi(ss[1]), mystoi(ss[2]), mystoi(ss[3]), mystoi(ss[4])));                    
                }
                pinss.push_back(pins);
            }

            
            cin >> s;
        }
        buses.push_back(bus(nb, np, nwv, pinss));
        cin >> s;
    }
    cin >> s;

    cin >> s >> nobstacles;
    for (int i = 0; i < nobstacles; ++i) {
        cin >> ss[0] >> ss[1] >> ss[2] >> ss[3] >> ss[4];
        obstacles.push_back(obstacle(NameToLayer[ss[0]], mystoi(ss[1]), mystoi(ss[2]), mystoi(ss[3]), mystoi(ss[4])));
    }
    
    cin >> s;
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
            if (curij[cj] != lij[cj]) {
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
            if (curij[cj] != lij[cj]) {
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
        while (1) {
            vector<int> v = extractBuses(v1, v2);
            // printv(v);
            if (v.size() == 0) break;
            sBus.push_back(v);
            for (auto & a : v) {
                b[a] = true;
            }
        }
        for (auto & k : v1) {
            if(!b[k.first]) {
                b[k.first] = true;
                sBus.push_back(vector<int>(1,k.first));
            }
        }
    }
}

int main(int argc, char** argv) {
    cout << argc << endl;
    for (auto i= 0; i < argc; ++i) {
        cout << argv[i] << endl;
    }
    freopen(argv[1], "r", stdin);
    freopen(argv[2], "w", stdout);

    readInput();
    // for (int i = 0; i < nbuses; ++i) {
    //     cout << i << ' ' << buses[i].pins.size() << endl;
    // }
    getSuperBus();
    getSuperBusPins3();
    getSBus();
    cout << nbuses << endl;
    int cnt = 0;
    for (auto v : sBus) {
        cout << v.size() << endl;
        for (auto vv : v) {
            cout << vv << ' ';
        }
        cout << endl;
        cnt += v.size();
    }
    cout << cnt << endl;

    // int k = superBus.size();
    // for (int i = 0; i < k; ++i) {
    //     cout << superBusPins[i].size() << endl;
    //     cout << superBusPins[i][0].size() << endl;
    //     for (auto a : superBusPins[i][0]) {
            
    //         cout << a.first << " " << a.second << '\t';
    //     }
    //     cout << endl;
    //     for (auto a : superBusPins[i][1]) {
    //         cout << a.first << " " << a.second << '\t';
    //     }
    //     cout << endl;
    //     auto b = LCS(superBusPins[i][0], superBusPins[i][1]);
    //     for (auto a : b) {
    //         cout << a.first << " " << a.second << '\t';
    //     }
    //     cout << endl;
    //     auto c = getRemoveBus(b);
    //     for (auto a : c) {
    //         cout << a << '\t';
    //     }
    //     cout << endl;
    // }
}