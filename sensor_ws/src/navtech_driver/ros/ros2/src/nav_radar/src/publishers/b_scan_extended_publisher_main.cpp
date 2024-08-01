#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "b_scan_extended_publisher.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Navtech::B_scan_extended_publisher>();

    RCLCPP_INFO(node->get_logger(), "Starting radar client");
    node->start();
    RCLCPP_INFO(node->get_logger(), "Radar client started");

    while (rclcpp::ok()) {
        spin(node);
    }

    RCLCPP_INFO(node->get_logger(), "Stopping radar client");
    node->stop();
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "Stopped radar client");
}