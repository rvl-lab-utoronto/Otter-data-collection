#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include "navtech_msgs/msg/radar_configuration_msg.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser_scan_subscriber_to_video.h"
#include "net_conversion.h"


std::shared_ptr<Laser_scan_subscriber_to_video> node{};

Laser_scan_subscriber_to_video::Laser_scan_subscriber_to_video() :Node{ "laser_scan_subscriber_to_video" }
{
    using std::placeholders::_1;

    rclcpp::QoS qos_radar_configuration_subscriber(radar_configuration_queue_size);
    qos_radar_configuration_subscriber.reliable();

    configuration_data_subscriber =
    Node::create_subscription<navtech_msgs::msg::RadarConfigurationMsg>(
        "radar_data/configuration_data",
        qos_radar_configuration_subscriber,
        std::bind(&Laser_scan_subscriber_to_video::configuration_data_callback, this, _1)
    );

    rclcpp::QoS qos_radar_laser_scan_subscriber(radar_laser_scan_queue_size);
    qos_radar_laser_scan_subscriber.reliable();

    laser_scan_subscriber =
    Node::create_subscription<sensor_msgs::msg::LaserScan>(
        "radar_data/laser_scan",
        qos_radar_laser_scan_subscriber,
        std::bind(&Laser_scan_subscriber_to_video::laser_scan_callback, this, _1)
    );
}


void Laser_scan_subscriber_to_video::configuration_data_callback(const navtech_msgs::msg::RadarConfigurationMsg::SharedPtr msg) const
{
    if (node->config_data_received) {
        return;
    }

    RCLCPP_INFO(Node::get_logger(), "Configuration Data recieved");

    if (msg->azimuth_samples) {
        node->azimuth_samples = msg->azimuth_samples;
        node->video_height = node->azimuth_samples;
        RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", node->azimuth_samples);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Azimuth Samples");
    }


    if (msg->encoder_size) {
        node->encoder_size = msg->encoder_size;
        RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", node->encoder_size);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Encoder Size");
    }


    if (msg->bin_size) {
        node->bin_size = msg->bin_size;
        RCLCPP_INFO(Node::get_logger(), "Bin Size: %f", node->bin_size);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Bin Size");
    }


    if (msg->range_in_bins) {
        node->video_width = msg->range_in_bins;
        RCLCPP_INFO(Node::get_logger(), "Range In Bins: %i", node->video_width);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Range In Bins");
    }


    if (msg->expected_rotation_rate) {
        node->expected_rotation_rate = msg->expected_rotation_rate;
        RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", node->expected_rotation_rate);
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

    node->video_writer.open("output_videos/laser_scan_output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), node->expected_rotation_rate, cv::Size(node->azimuth_samples, node->azimuth_samples), true);
    node->config_data_received = true;
}


void Laser_scan_subscriber_to_video::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
{
    if (!node->config_data_received) {
        RCLCPP_INFO(Node::get_logger(), "Configuration data not yet received");
        return;
    }

    RCLCPP_INFO(Node::get_logger(), "Laser Scan Received");
    time_t epoch = msg->header.stamp.sec;
    RCLCPP_INFO(Node::get_logger(), "Timestamp: %s", asctime(gmtime(&epoch)));
    RCLCPP_INFO(Node::get_logger(), "Start angle: %f", msg->angle_min * (180 / M_PI));
    RCLCPP_INFO(Node::get_logger(), "End angle: %f", msg->angle_max * (180 / M_PI));
    RCLCPP_INFO(Node::get_logger(), "Angle increment: %f", msg->angle_increment * (180 / M_PI));
    RCLCPP_INFO(Node::get_logger(), "Range min: %f", msg->range_min);
    RCLCPP_INFO(Node::get_logger(), "Range max: %f", msg->range_max);
    RCLCPP_INFO(Node::get_logger(), "Time increment: %f", msg->time_increment);
    RCLCPP_INFO(Node::get_logger(), "Scan time: %f", msg->scan_time);
    RCLCPP_INFO(Node::get_logger(), "Ranges: %li", msg->ranges.size());
    RCLCPP_INFO(Node::get_logger(), "Intensities: %li", msg->intensities.size());

    cv::Mat laser_scan_image{ cv::Size(azimuth_samples, azimuth_samples), CV_8UC1, cv::Scalar(0, 0) };
    for (int r{ 0 }; r < int(msg->ranges.size()); r++) {
        int index { static_cast<int>(msg->ranges[r] / bin_size )};
        int intensity { static_cast<int>(msg->intensities[r] )};
        if (index < azimuth_samples) {
            // Note - these cv:Points have been enhanced for visual purposes
            cv::circle(laser_scan_image, cv::Point(msg->ranges[r], r), 2 , cv::Scalar(intensity, intensity, intensity), cv::FILLED, 1);
        }
    }

    cv::Mat recovered_lin_polar_img{};
    cv::Point2f center{ static_cast<float>(laser_scan_image.cols / 2), static_cast<float>(laser_scan_image.rows / 2) };
    double max_radius{ std::min(center.y, center.x )};
    linearPolar(laser_scan_image, recovered_lin_polar_img, center, max_radius, cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS + cv::WARP_INVERSE_MAP);
    cv::Mat normalised_image{ cv::Size(azimuth_samples, azimuth_samples), CV_8UC1, cv::Scalar{0, 0} };
    normalize(recovered_lin_polar_img, normalised_image, 0, 255, cv::NORM_MINMAX);
    cv::Mat rotated_image{ cv::Size(azimuth_samples, azimuth_samples), CV_8UC1, cv::Scalar{0, 0} };
    rotate(normalised_image, rotated_image, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::Mat channels[3] { blank_image, rotated_image, blank_image };
    cv::Mat merged_data;
    merge(channels, 3, merged_data);
    node->video_writer.write(merged_data);
}