#include <iostream>
#include <string>
#include <queue>

using namespace std;
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
vector<T> LCS(const vector<T>& l, const vector<T>& r)
{
    int ll = l.size();
    int lr = r.size();
    cout << ll << endl;
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
    cout << f[100][100] << endl;
    int cl = ll;
    int cr = lr;
    int c = f[cl][cr];
    vector<T> ret(c);
    while(c > 0) {
        while(f[cl-1][cr] == c) --cl;
        while(f[cl][cr-1] == c) --cr;
        --c;
        ret[c] = l[cl-1];
        cout << l[cl-1] << endl;
        --cl;
        --cr;
    }
    return ret;
}

int main(){
    vector<int> v,vv;
    for(int i = 0; i <100; i++) {
        v.push_back(i);
        vv.push_back(i*2-3);
    }
    vector<int> vvv = LCS(v,vv);
    for (auto a : vvv) {
        cout << a << endl;
    }
}