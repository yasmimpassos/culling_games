#ifndef PATH_FINDER_H
#define PATH_FINDER_H

#include <bits/stdc++.h>

using Path = std::vector<int>;

Path astar_search(
    const std::vector<std::vector<int>>& adj,
    int start,
    int goal,
    int cols
);

std::vector<std::pair<int,int>> ids_to_coords(
    const Path& path_ids,
    int cols
);

#endif
