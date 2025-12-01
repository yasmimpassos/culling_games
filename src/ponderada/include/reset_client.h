#ifndef RESET_CLIENT_H
#define RESET_CLIENT_H

#include <bits/stdc++.h>
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/reset.hpp"

bool execute_reset(
    std::shared_ptr<rclcpp::Node> node,
    bool is_random = false,
    const std::string &map_name = ""
);

#endif
