#include <rclcpp/rclcpp.hpp>
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
#include "cg_interfaces/srv/reset.hpp"

#include <vector>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <set>
#include <stack>
#include <queue>
#include <cmath>
#include <algorithm>

using std::string;
using Pos = std::pair<int,int>;

constexpr int GRID_N = 29;
constexpr int GRID_SIZE = GRID_N * GRID_N;
constexpr std::chrono::milliseconds SHORT_DELAY(100);
constexpr std::chrono::milliseconds MOVE_DELAY(300);

static std::vector<std::vector<string>> world(GRID_N, std::vector<string>(GRID_N, "x"));
static int start_x = 1, start_y = 1;
static int cur_x = start_x, cur_y = start_y;

static std::set<Pos> visited;
static std::stack<Pos> dfs_stack;

static int astar_start_id = -1;
static int astar_target_id = -1;

class SensorNode : public rclcpp::Node {
    public:
        SensorNode() : Node("robot_sensors_node") {
            auto cb = [this](const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
                last_ = *msg;
                RCLCPP_DEBUG(this->get_logger(),
                    "sensores -> up:%s down:%s left:%s right:%s",
                    msg->up.c_str(), msg->down.c_str(), msg->left.c_str(), msg->right.c_str());
            };
            sub_ = this->create_subscription<cg_interfaces::msg::RobotSensors>(
                "/culling_games/robot_sensors", 10, cb);
        }

        cg_interfaces::msg::RobotSensors get_last() {
            return last_;
        }

    private:
        rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sub_;
        cg_interfaces::msg::RobotSensors last_;
};

class ResetClientNode : public rclcpp::Node {
    public:
        ResetClientNode() : Node("reset_client_node") {
            client_ = this->create_client<cg_interfaces::srv::Reset>("/reset");
        }
        void call_reset(bool randomize = false) {
            auto req = std::make_shared<cg_interfaces::srv::Reset::Request>();
            req->is_random = randomize;
            RCLCPP_INFO(this->get_logger(), "[RESET] pedindo reset (random=%s)", randomize ? "true" : "false");
            client_->async_send_request(req);
        }

    private:
        rclcpp::Client<cg_interfaces::srv::Reset>::SharedPtr client_;
};

class MoveClientNode : public rclcpp::Node {
    public:
        MoveClientNode() : Node("move_client_node") {
            client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        }

        void send(const std::string &dir) {
            auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
            req->direction = dir;
            RCLCPP_INFO(this->get_logger(), "[MOVE] enviando: %s", dir.c_str());
            client_->async_send_request(req);
        }

    private:
        rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_;
};


bool is_map_complete(const std::vector<std::vector<string>>& g) {
    for (const auto &row : g)
        for (const auto &c : row)
            if (c == "x") return false;
    return true;
}

void safe_set_cell(std::vector<std::vector<string>>& g, int y, int x, const string &val) {
    if (y >= 0 && y < (int)g.size() && x >= 0 && x < (int)g[0].size()) {
        if (g[y][x] == "x") g[y][x] = val;
    }
}

void feed_map_with_sensors(std::vector<std::vector<string>>& g, int cy, int cx, const cg_interfaces::msg::RobotSensors &m) {
    if (g[cy][cx] == "x") g[cy][cx] = "f";
    safe_set_cell(g, cy - 1, cx - 1, m.up_left);
    safe_set_cell(g, cy - 1, cx,     m.up);
    safe_set_cell(g, cy - 1, cx + 1, m.up_right);

    safe_set_cell(g, cy,     cx - 1, m.left);
    safe_set_cell(g, cy,     cx + 1, m.right);

    safe_set_cell(g, cy + 1, cx - 1, m.down_left);
    safe_set_cell(g, cy + 1, cx,     m.down);
    safe_set_cell(g, cy + 1, cx + 1, m.down_right);
}

void dump_map(const std::vector<std::vector<string>>& g) {
    std::cout << "\n=== MAPA RESULTANTE ===\n";
    for (const auto &row : g) {
        for (const auto &c : row) std::cout << c << ' ';
        std::cout << '\n';
    }
    std::cout << "=== FIM MAPA ===\n";
}

std::vector<Pos> neighbors_unvisited(const cg_interfaces::msg::RobotSensors &m, int y, int x) {
    std::vector<Pos> out;
    if (m.up == "f" && visited.find({y-1,x}) == visited.end()) out.push_back({y-1,x});
    if (m.right == "f" && visited.find({y,x+1}) == visited.end()) out.push_back({y,x+1});
    if (m.down == "f" && visited.find({y+1,x}) == visited.end()) out.push_back({y+1,x});
    if (m.left == "f" && visited.find({y,x-1}) == visited.end()) out.push_back({y,x-1});
    return out;
}

bool move_step(MoveClientNode &mnode, int target_y, int target_x, rclcpp::executors::MultiThreadedExecutor &exec) {
    std::string dir;
    if (target_y < cur_y) dir = "up";
    else if (target_y > cur_y) dir = "down";
    else if (target_x < cur_x) dir = "left";
    else if (target_x > cur_x) dir = "right";
    else return false;

    mnode.send(dir);

    std::this_thread::sleep_for(MOVE_DELAY);
    exec.spin_some();
    std::this_thread::sleep_for(MOVE_DELAY);
    exec.spin_some();

    if (dir == "up")    --cur_y;
    else if (dir == "down") ++cur_y;
    else if (dir == "left") --cur_x;
    else if (dir == "right") ++cur_x;

    return true;
}

void explore_with_dfs(MoveClientNode &mnode, rclcpp::executors::MultiThreadedExecutor &exec, std::shared_ptr<SensorNode> sensor_node) {
    visited.clear();
    while (!dfs_stack.empty()) dfs_stack.pop();

    visited.insert({cur_y, cur_x});
    dfs_stack.push({cur_y, cur_x});

    while (!is_map_complete(world)) {
        exec.spin_some();
        std::this_thread::sleep_for(SHORT_DELAY);

        auto msg = sensor_node->get_last();

        if (msg.up.empty() && msg.down.empty() && msg.left.empty() && msg.right.empty()) {
            RCLCPP_INFO(sensor_node->get_logger(), "[DFS] aguardando pacote de sensores...");
            continue;
        }

        feed_map_with_sensors(world, cur_y, cur_x, msg);

        if (is_map_complete(world)) {
            RCLCPP_INFO(sensor_node->get_logger(), "[DFS] mapa completo!");
            break;
        }

        auto nexts = neighbors_unvisited(msg, cur_y, cur_x);
        if (!nexts.empty()) {
            auto nxt = nexts.front();
            RCLCPP_INFO(sensor_node->get_logger(), "[DFS] movendo (%d,%d) -> (%d,%d) (visitados=%zu)",
                        cur_y, cur_x, nxt.first, nxt.second, visited.size());

            move_step(mnode, nxt.first, nxt.second, exec);
            visited.insert({cur_y, cur_x});
            dfs_stack.push({cur_y, cur_x});
        } else {
            if (dfs_stack.size() > 1) {
                dfs_stack.pop();
                if (!dfs_stack.empty()) {
                    Pos back = dfs_stack.top();
                    RCLCPP_INFO(sensor_node->get_logger(), "[DFS] backtrack de (%d,%d) para (%d,%d)",
                                cur_y, cur_x, back.first, back.second);
                    move_step(mnode, back.first, back.second, exec);
                }
            } else {
                break;
            }
        }
    }
}

std::vector<std::vector<int>> make_adj_matrix_from_world() {
    std::vector<std::vector<int>> mat(GRID_SIZE, std::vector<int>(GRID_SIZE, 0));
    int di[4] = {-1, 1, 0, 0};
    int dj[4] = {0, 0, -1, 1};

    for (int i = 0; i < GRID_N; ++i) {
        for (int j = 0; j < GRID_N; ++j) {
            string c = world[i][j];
            int id = i * GRID_N + j;
            if (c == "b") continue;
            if (c == "r") astar_start_id = id;
            if (c == "t") astar_target_id = id;
            for (int k = 0; k < 4; ++k) {
                int ni = i + di[k], nj = j + dj[k];
                if (ni < 0 || ni >= GRID_N || nj < 0 || nj >= GRID_N) continue;
                if (world[ni][nj] == "b") continue;
                int id2 = ni * GRID_N + nj;
                mat[id][id2] = 1;
            }
        }
    }
    return mat;
}

inline int manhattan(int id) {
    int ix = id / GRID_N;
    int jx = id % GRID_N;
    int tx = astar_target_id / GRID_N;
    int ty = astar_target_id % GRID_N;
    return std::abs(ix - tx) + std::abs(jx - ty);
}

std::vector<int> run_a_star(const std::vector<std::vector<int>> &adj) {
    const int N = GRID_SIZE;
    const int INF = 1e9;
    std::vector<int> g(N, INF), f(N, INF), from(N, -1);
    std::vector<char> closed(N, 0);

    using Pair = std::pair<int,int>; // (f, node)
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> open;

    g[astar_start_id] = 0;
    f[astar_start_id] = manhattan(astar_start_id);
    open.push({f[astar_start_id], astar_start_id});

    while (!open.empty()) {
        int cur = open.top().second;
        open.pop();
        if (closed[cur]) continue;
        closed[cur] = 1;

        if (cur == astar_target_id) break;

        for (int v = 0; v < N; ++v) {
            if (adj[cur][v]) {
                int tentative = g[cur] + 1;
                if (tentative < g[v]) {
                    g[v] = tentative;
                    from[v] = cur;
                    f[v] = tentative + manhattan(v);
                    open.push({f[v], v});
                }
            }
        }
    }
    return from;
}

std::vector<int> reconstruct_path_from_parents(int s, int t, const std::vector<int>& parent) {
    std::vector<int> path;
    int cur = t;
    while (cur != -1) {
        path.push_back(cur);
        if (cur == s) break;
        cur = parent[cur];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<string> ids_to_dirs(const std::vector<int>& path) {
    std::vector<string> moves;
    for (size_t k = 0; k + 1 < path.size(); ++k) {
        int a = path[k], b = path[k+1];
        int ai = a / GRID_N, aj = a % GRID_N;
        int bi = b / GRID_N, bj = b % GRID_N;
        if (bi == ai - 1 && bj == aj) moves.push_back("up");
        else if (bi == ai + 1 && bj == aj) moves.push_back("down");
        else if (bi == ai && bj == aj - 1) moves.push_back("left");
        else if (bi == ai && bj == aj + 1) moves.push_back("right");
    }
    return moves;
}

void execute_astar(MoveClientNode &mnode, rclcpp::executors::MultiThreadedExecutor &exec, std::shared_ptr<SensorNode> sensor_node) {
    RCLCPP_INFO(sensor_node->get_logger(), "[A*] construindo grafo...");
    auto adj = make_adj_matrix_from_world();

    if (astar_start_id < 0 || astar_target_id < 0) {
        RCLCPP_ERROR(sensor_node->get_logger(), "[A*] start ou target não definido (start=%d target=%d)", astar_start_id, astar_target_id);
        return;
    }

    RCLCPP_INFO(sensor_node->get_logger(), "[A*] start=%d target=%d heuristic(start)=%d",
                astar_start_id, astar_target_id, manhattan(astar_start_id));

    auto parents = run_a_star(adj);
    auto path = reconstruct_path_from_parents(astar_start_id, astar_target_id, parents);

    std::cout << "[A*] caminho (ids): ";
    for (auto id : path) std::cout << id << ' ';
    std::cout << '\n';

    auto moves = ids_to_dirs(path);
    std::cout << "[A*] movimentos: ";
    for (const auto &m : moves) std::cout << '"' << m << '"' << ", ";
    std::cout << '\n';

    RCLCPP_INFO(sensor_node->get_logger(), "[A*] iniciando navegação de %zu passos", moves.size());
    for (size_t i = 0; i < moves.size(); ++i) {
        RCLCPP_INFO(sensor_node->get_logger(), "[A*] passo %zu/%zu: %s", i+1, moves.size(), moves[i].c_str());
        mnode.send(moves[i]);
        std::this_thread::sleep_for(MOVE_DELAY);
        exec.spin_some();
    }
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto sensor = std::make_shared<SensorNode>();
    auto mover = std::make_shared<MoveClientNode>();
    auto reset = std::make_shared<ResetClientNode>();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(sensor);
    exec.add_node(mover);
    exec.add_node(reset);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    exec.spin_some();

    world[cur_y][cur_x] = "r";

    explore_with_dfs(*mover, exec, sensor);

    dump_map(world);
    RCLCPP_INFO(sensor->get_logger(), "Mapeamento finalizado.");

    reset->call_reset(false);

    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    exec.spin_some();

    cur_x = start_x;
    cur_y = start_y;

    execute_astar(*mover, exec, sensor);

    rclcpp::shutdown();
    return 0;
}
