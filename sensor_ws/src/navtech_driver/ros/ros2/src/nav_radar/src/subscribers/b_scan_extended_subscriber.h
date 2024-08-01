#include <rclcpp/rclcpp.hpp>

#include "navtech_msgs/msg/radar_b_scan_msg.hpp"

class b_scan_extended_subscriber : public ::rclcpp::Node
{
public:
    b_scan_extended_subscriber();

private:

    constexpr static int b_scan_subscriber_queue_size { 400 };

    void b_scan_msg_callback(const navtech_msgs::msg::RadarBScanMsg::SharedPtr msg) const;

    rclcpp::Subscription<navtech_msgs::msg::RadarBScanMsg>::SharedPtr b_scan_msg_subscriber;
};