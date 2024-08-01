#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "b_scan_extended_subscriber.h"

b_scan_extended_subscriber::b_scan_extended_subscriber() : Node { "b_scan_extended_subscriber" }
{
    using std::placeholders::_1;
    using namespace std::chrono_literals;

    rclcpp::QoS qos_b_scan_msg_subscriber(b_scan_subscriber_queue_size);
    qos_b_scan_msg_subscriber.reliable();

    b_scan_msg_subscriber =
    Node::create_subscription<navtech_msgs::msg::RadarBScanMsg>(
        "radar_data/b_scan_msg",
        qos_b_scan_msg_subscriber,
        std::bind(&b_scan_extended_subscriber::b_scan_msg_callback, this, _1)
    );
}


void b_scan_extended_subscriber::b_scan_msg_callback(const navtech_msgs::msg::RadarBScanMsg::SharedPtr msg) const
{
    auto incoming_b_scan = msg->b_scan_img;

    RCLCPP_INFO(
        Node::get_logger(),
        "Received B Scan with and dimensions %ix%i in %s encoding",
        incoming_b_scan.height,
        incoming_b_scan.width,
        incoming_b_scan.encoding
    );

}