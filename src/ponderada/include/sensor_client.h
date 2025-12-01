#ifndef SENSOR_CLIENT_H
#define SENSOR_CLIENT_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
#include <bits/stdc++.h>

class SensorClient : public rclcpp::Node {
public:
    SensorClient();
    cg_interfaces::msg::RobotSensors::SharedPtr get_last_msg();

private:
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr subscription_;
    cg_interfaces::msg::RobotSensors::SharedPtr last_msg_;
};

#endif  