#include "sensor_client.h"

using namespace std;

SensorClient::SensorClient(): Node("sensor_client_node"){
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    subscription_ = this->create_subscription<cg_interfaces::msg::RobotSensors>(
        "/culling_games/robot_sensors",
        qos,
        [this](const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
            last_msg_ = msg;
        }
    );
}

cg_interfaces::msg::RobotSensors::SharedPtr SensorClient::get_last_msg() {
    return last_msg_;
}

