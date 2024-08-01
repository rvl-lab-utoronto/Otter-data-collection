#include <rclcpp/rclcpp.hpp>

#include "navtech_msgs/msg/radar_configuration_msg.hpp"
#include "navtech_msgs/msg/radar_fft_data_msg.hpp"
#include "colossus_and_camera_subscriber_to_video.h"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    node = std::make_shared<Colossus_and_camera_subscriber_to_video>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}