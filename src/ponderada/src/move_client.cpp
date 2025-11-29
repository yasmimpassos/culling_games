#include "move_client.h"

using namespace std;

bool execute_move_sequence(shared_ptr<rclcpp::Node> node,const vector<string>& moves) {
    auto client = node->create_client<cg_interfaces::srv::MoveCmd>("/move_command");

    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "ROS interrompido enquanto esperava /move_command");
            return false;
        }
    }

    for (const auto &dir : moves) {
        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        req->direction = dir;

        auto fut = client->async_send_request(req);
        if (rclcpp::spin_until_future_complete(node, fut) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "Falha ao chamar /move_command para direção %s", dir.c_str());
            return false;
        }

        this_thread::sleep_for(chrono::milliseconds(200));
    }

    return true;
}
