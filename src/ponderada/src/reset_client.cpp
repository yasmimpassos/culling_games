#include "reset_client.h"

using namespace std;

bool execute_reset(shared_ptr<rclcpp::Node> node, bool is_random, const string &map_name) {
    auto client = node->create_client<cg_interfaces::srv::Reset>("/reset");

    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(),
                         "ROS interrompido enquanto aguardava /reset");
            return false;
        }
    }

    auto req = std::make_shared<cg_interfaces::srv::Reset::Request>();
    req->is_random = is_random;
    req->map_name  = map_name;

    auto fut = client->async_send_request(req);

    if (rclcpp::spin_until_future_complete(node, fut)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Falha ao chamar /reset");
        return false;
    }

    auto res = fut.get();

    if (!res->success) {
        RCLCPP_WARN(node->get_logger(),
                    "Reset retornou false. Mapa atual permanece: %s",
                    res->loaded_map_name.c_str());
        return false;
    }

    RCLCPP_INFO(node->get_logger(),
                "Reset executado com sucesso! Mapa carregado: %s",
                res->loaded_map_name.c_str());

    return true;
}
