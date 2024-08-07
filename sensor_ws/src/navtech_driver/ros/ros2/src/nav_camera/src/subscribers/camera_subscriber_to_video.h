#include <rclcpp/rclcpp.hpp>

#include "navtech_msgs/msg/camera_configuration_message.hpp"
#include "sensor_msgs/msg/image.hpp"

class Camera_subscriber_to_video : public ::rclcpp::Node {
public:
    Camera_subscriber_to_video();

private:
    constexpr static int camera_configuration_queue_size{ 1 };
    constexpr static int camera_image_queue_size{ 25 };

    void configuration_data_callback(const navtech_msgs::msg::CameraConfigurationMessage::SharedPtr data) const;
    void camera_image_callback(const sensor_msgs::msg::Image::SharedPtr data) const;

    rclcpp::Subscription<navtech_msgs::msg::CameraConfigurationMessage>::SharedPtr camera_configuration_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_data_subscriber;
};