#ifndef EXPLORER_H
#define EXPLORER_H

#include <bits/stdc++.h>

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
#include "sensor_client.h"
#include "move_client.h"

using Pos = std::pair<int,int>;

class SensorClient;

std::optional<Pos> explore_with_dfs(
    std::shared_ptr<rclcpp::Node> node,
    rclcpp::executors::MultiThreadedExecutor &exec,
    std::shared_ptr<SensorClient> sensor_node,
    std::vector<std::vector<std::string>>& world,
    Pos start_pos
);

#endif
