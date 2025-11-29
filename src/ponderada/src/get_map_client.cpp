#include "get_map_client.h"

using namespace std;

shared_ptr<cg_interfaces::srv::GetMap::Response> get_map_client(shared_ptr<rclcpp::Node> node){
    auto client = node->create_client<cg_interfaces::srv::GetMap>("get_map");

    while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(node->get_logger(), "Esperando get_map estar disponivel!");
    }

    auto req = make_shared<cg_interfaces::srv::GetMap::Request>();
    auto fut = client->async_send_request(req);

    if (rclcpp::spin_until_future_complete(node, fut) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Mapa recebido com sucesso!");
        return fut.get();
    }

    RCLCPP_ERROR(node->get_logger(), "Não foi possivel pegar o mapa!");
    return nullptr;
}