#ifndef GET_MAP_CLIENT_H
#define GET_MAP_CLIENT_H

#include <bits/stdc++.h>
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"

std::shared_ptr<cg_interfaces::srv::GetMap::Response> get_map_client(
    std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr<rclcpp::Client<cg_interfaces::srv::GetMap>> client
);

#endif
