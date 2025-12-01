#include "rclcpp/rclcpp.hpp"
#include "sensor_client.h"
#include "explorer.h"
#include "map_printer.h"
#include "reset_client.h"
#include "graph_adj_matrix.h"
#include "path_finder.h"
#include "path_moves.h"
#include "move_client.h"

#include <bits/stdc++.h>

using namespace std;

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto sensor_node = make_shared<SensorClient>();
    auto move_node = make_shared<rclcpp::Node>("move_node");
    auto main_node = make_shared<rclcpp::Node>("explorer_main");

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(sensor_node);

    const int rows = 29, cols = 29;
    vector<vector<string>> matrix(rows, vector<string>(cols, "x"));

    Pos start = {1, 1};
    matrix[start.first][start.second] = "r";

    RCLCPP_INFO(main_node->get_logger(), "Aguardando primeiro pacote de sensores...");
    while (rclcpp::ok() && !sensor_node->get_last_msg()) {
        exec.spin_some();
        this_thread::sleep_for(chrono::milliseconds(30));
    }

    RCLCPP_INFO(main_node->get_logger(), "Iniciando exploracao DFS...");

    auto result = explore_with_dfs(
        move_node,
        exec,
        sensor_node,
        matrix,
        start
    );

    print_map(matrix);

    execute_reset(move_node);

    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    exec.spin_some();

    vector<vector<int>> adj;
    int start_id, goal_id;

    build_adjacency_matrix(matrix, adj, start_id, goal_id);

    if (start_id < 0 || goal_id < 0) {
        RCLCPP_ERROR(move_node->get_logger(), "Não foi encontrado start_id ou goal_id no mapa!");
        rclcpp::shutdown();
        return 1;
    }

    auto path_ids = astar_search(adj, start_id, goal_id, cols);

    if (path_ids.empty()) {
        RCLCPP_WARN(move_node->get_logger(), "Nenhum caminho encontrado do start ao goal!");
        rclcpp::shutdown();
        return 1;
    }

    auto path_coords = ids_to_coords(path_ids, cols);

    RCLCPP_INFO(move_node->get_logger(), "Caminho encontrado com %zu nós:", path_coords.size());
    for (auto &p : path_coords) cout << "(" << p.first << "," << p.second << ") ";
    cout << endl;

    auto moves = path_to_moves(path_coords);
    for (auto &m : moves) cout << m << " ";
    cout << endl;

    bool ok = execute_move_sequence(move_node, moves);

    if (!ok)
        RCLCPP_ERROR(move_node->get_logger(), "Falha ao executar a sequência de movimentos!");
    else
        RCLCPP_INFO(move_node->get_logger(), "Sequência de movimentos enviada com sucesso.");

    rclcpp::shutdown();
    return 0;
}