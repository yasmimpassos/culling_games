#include <bits/stdc++.h>
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "get_map_client.h"
#include "map_graph.h"
#include "map_printer.h"
#include "graph_adj_matrix.h"
#include "path_finder.h"

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

    vector<vector<int>> adj;
    int start_id, goal_id;

    build_adjacency_matrix(matrix, adj, start_id, goal_id);

    if (start_id < 0 || goal_id < 0) {
        RCLCPP_ERROR(node->get_logger(), "Não foi encontrado start_id ou goal_id no mapa!");
        rclcpp::shutdown();
        return 1;
    }

    auto path_ids = astar_search(adj, start_id, goal_id, cols);

    if (path_ids.empty()) {
        RCLCPP_WARN(node->get_logger(), "Nenhum caminho encontrado do start ao goal!");
    } else {
        auto path_coords = ids_to_coords(path_ids, cols);

        RCLCPP_INFO(node->get_logger(), "Caminho encontrado com %zu nós:", path_coords.size());
        for (auto & [r, c] : path_coords) {
            cout << "(" << r << "," << c << ") ";
        }
        cout << endl;
    }

    rclcpp::shutdown();
    return 0;
}