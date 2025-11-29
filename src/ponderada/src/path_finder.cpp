#include "path_finder.h"
#include <bits/stdc++.h>

using namespace std;

static int heuristic(int id, int goal, int cols) {
    int r1 = id / cols;
    int c1 = id % cols;
    int r2 = goal / cols;
    int c2 = goal % cols;
    return abs(r1 - r2) + abs(c1 - c2);
}

Path astar_search(const vector<vector<int>>& adj, int start, int goal, int cols) {

    int N = adj.size();
    const int INF = numeric_limits<int>::max();

    vector<int> gscore(N, INF), fscore(N, INF), parent(N, -1);
    vector<bool> closed(N, false);

    priority_queue<pair<int,int>,vector<pair<int,int>>,greater<pair<int,int>>> open_set;

    gscore[start] = 0;
    fscore[start] = heuristic(start, goal, cols);
    open_set.push({ fscore[start], start });

    while (!open_set.empty()) {
        auto [fcur, u] = open_set.top();
        open_set.pop();

        if (closed[u]) continue;
        closed[u] = true;

        if (u == goal) {
            Path path;
            for (int at = goal; at != -1; at = parent[at]) {
                path.push_back(at);
            }
            reverse(path.begin(), path.end());
            return path;
        }

        for (int v = 0; v < N; ++v) {
            int w = adj[u][v];
            if (w == 0) continue;
            if (closed[v]) continue;

            int tentative_g = gscore[u] + w;
            if (tentative_g < gscore[v]) {
                parent[v] = u;
                gscore[v] = tentative_g;
                fscore[v] = tentative_g + heuristic(v, goal, cols);
                open_set.push({ fscore[v], v });
            }
        }
    }

    return Path();
}

vector<pair<int,int>> ids_to_coords(const Path& path_ids, int cols) {
    vector<pair<int,int>> coords;
    coords.reserve(path_ids.size());
    for (int id : path_ids) {
        int r = id / cols;
        int c = id % cols;
        coords.emplace_back(r, c);
    }
    return coords;
}