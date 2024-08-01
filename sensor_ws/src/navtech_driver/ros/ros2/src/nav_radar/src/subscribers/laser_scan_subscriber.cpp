#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <math.h>

#include "navtech_msgs/msg/radar_configuration_msg.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser_scan_subscriber.h"
#include "net_conversion.h"


Laser_scan_subscriber::Laser_scan_subscriber() : Node{ "laser_scan_subscriber" }
{
    using std::placeholders::_1;

    rclcpp::QoS qos_radar_configuration_subscriber(radar_configuration_queue_size);
    qos_radar_configuration_subscriber.reliable();

    configuration_data_subscriber = 
    Node::create_subscription<navtech_msgs::msg::RadarConfigurationMsg>(
        "radar_data/configuration_data",
        qos_radar_configuration_subscriber,
        std::bind(&Laser_scan_subscriber::configuration_data_callback, this, _1)
    );

    rclcpp::QoS qos_radar_laser_scan_subscriber(radar_laser_scan_queue_size);
    qos_radar_laser_scan_subscriber.reliable();

    laser_scan_subscriber =
    Node::create_subscription<sensor_msgs::msg::LaserScan>(
        "radar_data/laser_scan",
        qos_radar_laser_scan_subscriber,
        std::bind(&Laser_scan_subscriber::laser_scan_callback, this, _1)
    );
}


void Laser_scan_subscriber::configuration_data_callback(const navtech_msgs::msg::RadarConfigurationMsg::SharedPtr msg) const
{
    RCLCPP_INFO(Node::get_logger(), "Configuration Data recieved");


    if (msg->azimuth_samples) {
        RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", msg->azimuth_samples);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Azimuth Samples");
    }


    if (msg->encoder_size) {
        RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", msg->encoder_size);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Encoder Size");
    }


    if (msg->bin_size) {
        RCLCPP_INFO(Node::get_logger(), "Bin Size: %f", msg->bin_size);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Bin Size");
    }


    if (msg->range_in_bins) {
        RCLCPP_INFO(Node::get_logger(), "Range In Bins: %i", msg->range_in_bins);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Range In Bins");
    }


    if (msg->expected_rotation_rate) {
        RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", msg->expected_rotation_rate);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Expected Rotation Rate");
    }


    if (msg->range_gain) {
        RCLCPP_INFO(Node::get_logger(), "Range Gain: %f", msg->range_gain);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Range Gain");
    }


    if (msg->range_offset) {
        RCLCPP_INFO(Node::get_logger(), "Range Offset: %f", msg->range_offset);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Range Offset");
    }
}


void Laser_scan_subscriber::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
{
    RCLCPP_INFO(Node::get_logger(), "Laser Scan Received");
    time_t epoch = msg->header.stamp.sec;
    RCLCPP_INFO(Node::get_logger(), "Timestamp: %s", asctime(gmtime(&epoch)));
    RCLCPP_INFO(Node::get_logger(), "Start angle: %f", msg->angle_min * (180 / M_PI));
    RCLCPP_INFO(Node::get_logger(), "End angle: %f", msg->angle_max * (180 / M_PI));
    RCLCPP_INFO(Node::get_logger(), "Angle increment: %f", msg->angle_increment * (180 / M_PI));
    RCLCPP_INFO(Node::get_logger(), "Ranges: %li", msg->ranges.size());
    RCLCPP_INFO(Node::get_logger(), "Intensities: %li", msg->intensities.size());
}
