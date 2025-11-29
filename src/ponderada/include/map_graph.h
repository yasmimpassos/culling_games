#ifndef MAP_GRAPH_H
#define MAP_GRAPH_H

#include <bits/stdc++.h>
#include "cg_interfaces/srv/get_map.hpp"

void map_graph(
    const std::shared_ptr<cg_interfaces::srv::GetMap::Response>& response,
    std::vector<std::vector<std::string>>& matrix_out
);

#endif
