#ifndef MOVE_CLIENT_H
#define MOVE_CLIENT_H

#include <bits/stdc++.h>

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

bool execute_move_sequence(
    std::shared_ptr<rclcpp::Node> node,
    const std::vector<std::string>& moves
);

#endif
