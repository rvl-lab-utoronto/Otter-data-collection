#include <rclcpp/rclcpp.hpp>
#include "b_scan_extended_subscriber.h"

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<b_scan_extended_subscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}