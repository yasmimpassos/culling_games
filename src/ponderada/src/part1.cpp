#include <bits/stdc++.h>
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "get_map_client.h"
#include "map_graph.h"
#include "map_printer.h"

using namespace std;

int main(int argc, char * argv[]){

    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("map_client_node");
    auto client = node->create_client<cg_interfaces::srv::GetMap>("get_map");

    auto response = get_map_client(node, client);

    if (!response) {
        RCLCPP_ERROR(node->get_logger(), "Falha ao obter o mapa!");
        rclcpp::shutdown();
        return 1;
    }

    if (response->occupancy_grid_shape.size() != 2) {
        RCLCPP_ERROR(node->get_logger(), "Shape informado invalido!");
        rclcpp::shutdown();
        return 1;
    }


    int rows = response->occupancy_grid_shape[0];
    int cols = response->occupancy_grid_shape[1];

    vector<vector<string>> matrix;
    matrix.assign(rows, vector<string>(cols, ""));

    map_graph(response, matrix);

    if (matrix.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Matriz retornada vazia / inválida!");
        rclcpp::shutdown();
        return 1;
    }

    print_map(matrix);

    rclcpp::shutdown();
    return 0;
}