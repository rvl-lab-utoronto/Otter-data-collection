#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <exception>
#include <cassert>

#include "navtech_msgs/msg/radar_configuration_msg.hpp"
#include "navtech_msgs/msg/radar_fft_data_msg.hpp"
#include "colossus_subscriber.h"
#include "net_conversion.h"


Colossus_subscriber::Colossus_subscriber() : Node{ "colossus_subscriber" }
{
    using std::placeholders::_1;

    rclcpp::QoS qos_radar_configuration_subscriber(radar_configuration_queue_size);
    qos_radar_configuration_subscriber.reliable();

    configuration_data_subscriber = 
    Node::create_subscription<navtech_msgs::msg::RadarConfigurationMsg>(
        "radar_data/configuration_data",
        qos_radar_configuration_subscriber,
        std::bind(&Colossus_subscriber::configuration_data_callback, this, _1)
    );

    rclcpp::QoS qos_radar_fft_subscriber(radar_fft_queue_size);
    qos_radar_fft_subscriber.reliable();

    fft_data_subscriber =
    Node::create_subscription<navtech_msgs::msg::RadarFftDataMsg>(
        "radar_data/fft_data",
        qos_radar_fft_subscriber,
        std::bind(&Colossus_subscriber::fft_data_callback, this, _1)
    );
}


void Colossus_subscriber::configuration_data_callback(const navtech_msgs::msg::RadarConfigurationMsg::SharedPtr msg) const
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


void Colossus_subscriber::fft_data_callback(const navtech_msgs::msg::RadarFftDataMsg::SharedPtr msg) const
{
    RCLCPP_INFO(Node::get_logger(), "FFT Data Received");


    if (msg->angle) {
        RCLCPP_INFO(Node::get_logger(), "Angle: %f", msg->angle);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Angle");
    }


    if (msg->azimuth) {
        RCLCPP_INFO(Node::get_logger(), "Azimuth: %i", msg->azimuth);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Azimuth");
    }


    if (msg->sweep_counter) {
        RCLCPP_INFO(Node::get_logger(), "Sweep Counter: %i", msg->sweep_counter);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Sweep Counter");
    }


    if (msg->ntp_seconds) {
        RCLCPP_INFO(Node::get_logger(), "NTP Seconds: %i", msg->ntp_seconds);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: NTP Seconds");
    }


    if (msg->ntp_split_seconds) {
        RCLCPP_INFO(Node::get_logger(), "NTP Split Seconds: %i", msg->ntp_split_seconds);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: NTP SPlit Seconds");
    }


    if (msg->data_length) {
        RCLCPP_INFO(Node::get_logger(), "Data Length: %i", msg->data_length);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Data Length");
    }
}
