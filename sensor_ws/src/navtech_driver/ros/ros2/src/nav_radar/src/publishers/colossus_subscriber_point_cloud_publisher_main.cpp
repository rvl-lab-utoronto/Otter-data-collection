#include <rclcpp/rclcpp.hpp>
#include "navtech_msgs/msg/radar_configuration_msg.hpp"
#include "navtech_msgs/msg/radar_fft_data_msg.hpp"
#include "colossus_subscriber_point_cloud_publisher.h"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Colossus_subscriber_point_cloud_publisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}