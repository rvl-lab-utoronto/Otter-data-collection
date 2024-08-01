#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "navtech_msgs/msg/radar_configuration_msg.hpp"
#include "navtech_msgs/msg/radar_fft_data_msg.hpp"
#include "navtech_msgs/msg/camera_configuration_message.hpp"
#include "colossus_and_camera_publisher.h"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Colossus_and_camera_publisher>();

    RCLCPP_INFO(node->get_logger(), "Starting radar client");
    node->start();
    RCLCPP_INFO(node->get_logger(), "Radar client started");

    RCLCPP_INFO(node->get_logger(), "Starting camera publisher");
    RCLCPP_INFO(node->get_logger(), "URL: %s", node->get_camera_url().c_str());

    std::shared_ptr<Video_capture_manager> vid_cap_manager = std::make_shared<Video_capture_manager>();

    auto ret = vid_cap_manager->connect_to_camera(node->get_camera_url());

    if (ret) {

        cv::Mat image{ };

        while (rclcpp::ok()) {

            image = vid_cap_manager->get_latest_frame();
            node->camera_image_handler(image);
            spin_some(node);
        }
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down");
    vid_cap_manager->disconnect_from_camera();

    RCLCPP_INFO(node->get_logger(), "Stopping radar client");
    node->stop();
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "Stopped radar client");
}