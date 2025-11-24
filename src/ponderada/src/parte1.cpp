#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/get_map.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>

std::vector<std::vector<char>> grid;
int start_id;
int goal_id;
using Pair = std::pair<int, int>;
constexpr std::chrono::milliseconds MOVE_DELAY(250);

class MapClient : public rclcpp::Node {
public:
    MapClient() : Node("map_client") {
        client_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");
    }

    std::shared_ptr<cg_interfaces::srv::GetMap::Response> request_map() {
        RCLCPP_INFO(this->get_logger(), "Esperando serviço /get_map...");
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Aguardando disponibilidade do serviço...");
        }
        auto req = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        auto fut = client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Mapa recebido com sucesso.");
            return fut.get();
        }
        RCLCPP_ERROR(this->get_logger(), "Erro ao requisitar o mapa.");
        return nullptr;
    }

private:
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr client_;
};

class MoveClient : public rclcpp::Node {
public:
    MoveClient() : Node("move_client_node") {
        client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    }

    void send_move(const std::string &dir) {
        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        req->direction = dir;
        client_->async_send_request(req);
        RCLCPP_INFO(this->get_logger(), "Enviado comando de movimento: %s", dir.c_str());
    }

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_;
};

void build_grid(const std::shared_ptr<cg_interfaces::srv::GetMap::Response> res) {
    int h = res->occupancy_grid_shape[0];
    int w = res->occupancy_grid_shape[1];
    grid.assign(h, std::vector<char>(w));
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            grid[i][j] = res->occupancy_grid_flattened[i * w + j][0];
        }
    }
}

void show_grid() {
    std::cout << "\nMAPA:\n\n";
    for (auto &r : grid) {
        for (char c : r) std::cout << c << ' ';
        std::cout << '\n';
    }
}

std::vector<std::vector<int>> make_adj() {
    const int N = 29 * 29;
    std::vector<std::vector<int>> m(N, std::vector<int>(N, 0));
    int dy[4] = {-1, 1, 0, 0};
    int dx[4] = {0, 0, -1, 1};
    for (int i = 0; i < 29; i++) {
        for (int j = 0; j < 29; j++) {
            char c = grid[i][j];
            int id = i * 29 + j;
            if (c == 'b') continue;
            if (c == 'r') start_id = id;
            if (c == 't') goal_id = id;
            for (int k = 0; k < 4; k++) {
                int ny = i + dy[k], nx = j + dx[k];
                if (ny < 0 || ny >= 29 || nx < 0 || nx >= 29) continue;
                if (grid[ny][nx] == 'b') continue;
                int id2 = ny * 29 + nx;
                m[id][id2] = 1;
            }
        }
    }
    return m;
}

int h(int id) {
    int y = id / 29, x = id % 29;
    int ty = goal_id / 29, tx = goal_id % 29;
    return std::abs(y - ty) + std::abs(x - tx);
}

std::vector<int> astar(const std::vector<std::vector<int>> &adj) {
    const int N = 29 * 29;
    std::vector<int> g(N, 1e9), f(N, 1e9), parent(N, -1);
    std::vector<bool> used(N, false);
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> pq;
    g[start_id] = 0;
    f[start_id] = h(start_id);
    pq.push({f[start_id], start_id});
    while (!pq.empty()) {
        int cur = pq.top().second;
        pq.pop();
        if (used[cur]) continue;
        used[cur] = true;
        if (cur == goal_id) break;
        for (int v = 0; v < N; v++) {
            if (adj[cur][v] == 1) {
                int temp = g[cur] + 1;
                if (temp < g[v]) {
                    parent[v] = cur;
                    g[v] = temp;
                    f[v] = temp + h(v);
                    pq.push({f[v], v});
                }
            }
        }
    }
    return parent;
}

std::vector<int> make_path(int s, int t, const std::vector<int> &p) {
    std::vector<int> path;
    int cur = t;
    while (cur != -1) {
        path.push_back(cur);
        if (cur == s) break;
        cur = p[cur];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<std::string> ids_to_moves(const std::vector<int> &path) {
    std::vector<std::string> mv;
    for (int i = 0; i < (int)path.size() - 1; i++) {
        int a = path[i], b = path[i + 1];
        int ay = a / 29, ax = a % 29;
        int by = b / 29, bx = b % 29;
        if (by == ay - 1 && bx == ax) mv.push_back("up");
        else if (by == ay + 1 && bx == ax) mv.push_back("down");
        else if (by == ay && bx == ax - 1) mv.push_back("left");
        else if (by == ay && bx == ax + 1) mv.push_back("right");
    }
    return mv;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto map_node = std::make_shared<MapClient>();
    auto move_node = std::make_shared<MoveClient>();

    auto res = map_node->request_map();
    if (!res) {
        rclcpp::shutdown();
        return 1;
    }

    build_grid(res);
    show_grid();

    auto adj = make_adj();
    auto came = astar(adj);
    auto path = make_path(start_id, goal_id, came);
    auto moves = ids_to_moves(path);

    std::cout << "\nExecutando movimentos:\n";
    for (auto &m : moves) {
        move_node->send_move(m);
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        rclcpp::spin_some(move_node);
    }

    rclcpp::shutdown();
    return 0;
}
