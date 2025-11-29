#ifndef GRAPH_ADJ_MATRIX
#define GRAPH_ADJ_MATRIX

#include <bits/stdc++.h>

void build_adjacency_matrix(
    const std::vector<std::vector<std::string>>& grid,
    std::vector<std::vector<int>>& adj_matrix,
    int& start_id,
    int& goal_id
);

#endif
