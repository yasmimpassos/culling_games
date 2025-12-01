#include "explorer.h"

using namespace std;
using cg_interfaces::msg::RobotSensors;

static const chrono::milliseconds SHORT_DELAY(50);

static set<Pos> visited;
static stack<Pos> dfs_stack;

static int cur_y = 0;
static int cur_x = 0;

bool is_complete(const vector<vector<string>>& g) {
    for (auto &row : g)
        for (auto &c : row)
            if (c == "x") return false;
    return true;
}

void safe_set_cell(vector<vector<string>>& g, int y, int x, const string &val) {
    if (y >= 0 && y < (int)g.size() && x >= 0 && x < (int)g[0].size()) {
        if (g[y][x] == "x") g[y][x] = val;
    }
}

void feed_map_with_sensors(vector<vector<string>>& g, int cy, int cx, const RobotSensors &m) {

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

vector<Pos> neighbors_unvisited(const RobotSensors &m, int y, int x) {
    vector<Pos> out;
    if (m.up == "f"    && visited.count({y-1,x}) == 0) out.push_back({y-1,x});
    if (m.right == "f" && visited.count({y,x+1}) == 0) out.push_back({y,x+1});
    if (m.down == "f"  && visited.count({y+1,x}) == 0) out.push_back({y+1,x});
    if (m.left == "f"  && visited.count({y,x-1}) == 0) out.push_back({y,x-1});
    return out;
}

static vector<string> direction_for_step(int ty, int tx) {

    if (ty < cur_y) return {"up"};
    if (ty > cur_y) return {"down"};
    if (tx < cur_x) return {"left"};
    if (tx > cur_x) return {"right"};

    return {"invalid"};
}

optional<Pos> explore_with_dfs(
    shared_ptr<rclcpp::Node> node,
    rclcpp::executors::MultiThreadedExecutor &exec,
    shared_ptr<SensorClient> sensor_node,
    vector<vector<string>>& world,
    Pos start_pos
) {
    visited.clear();
    while (!dfs_stack.empty()) dfs_stack.pop();

    cur_y = start_pos.first;
    cur_x = start_pos.second;

    visited.insert({cur_y, cur_x});
    dfs_stack.push({cur_y, cur_x});

    while (!is_complete(world)) {

        exec.spin_some();
        this_thread::sleep_for(SHORT_DELAY);

        auto msg_ptr = sensor_node->get_last_msg();
        if (!msg_ptr) {
            RCLCPP_INFO(node->get_logger(), "[DFS] aguardando pacote de sensores...");
            continue;
        }

        RobotSensors msg = *msg_ptr;

        if (msg.up.empty() && msg.down.empty() &&
            msg.left.empty() && msg.right.empty()) 
        {
            RCLCPP_INFO(node->get_logger(), "[DFS] aguardando pacote de sensores...");
            continue;
        }

        feed_map_with_sensors(world, cur_y, cur_x, msg);

        if (is_complete(world)) {
            RCLCPP_INFO(node->get_logger(), "[DFS] mapa completo!");
            break;
        }

        auto nexts = neighbors_unvisited(msg, cur_y, cur_x);

        if (!nexts.empty()) {
            auto nxt = nexts.front();

            RCLCPP_INFO(node->get_logger(),
                "[DFS] movendo (%d,%d) -> (%d,%d) (visitados=%zu)",
                cur_y, cur_x, nxt.first, nxt.second, visited.size());

            auto move_cmd = direction_for_step(nxt.first, nxt.second);
            execute_move_sequence(node, move_cmd);

            cur_y = nxt.first;
            cur_x = nxt.second;

            visited.insert({cur_y, cur_x});
            dfs_stack.push({cur_y, cur_x});
        }
        else {
            if (dfs_stack.size() > 1) {
                dfs_stack.pop();

                if (!dfs_stack.empty()) {
                    Pos back = dfs_stack.top();

                    RCLCPP_INFO(node->get_logger(),
                        "[DFS] backtrack de (%d,%d) para (%d,%d)",
                        cur_y, cur_x, back.first, back.second);

                    auto mv = direction_for_step(back.first, back.second);
                    execute_move_sequence(node, mv);

                    cur_y = back.first;
                    cur_x = back.second;
                }
            }
            else {
                break;
            }
        }
    }

    for (int y = 0; y < (int)world.size(); y++)
        for (int x = 0; x < (int)world[0].size(); x++)
            if (world[y][x] == "t")
                return Pos(y,x);

    return nullopt;
}
