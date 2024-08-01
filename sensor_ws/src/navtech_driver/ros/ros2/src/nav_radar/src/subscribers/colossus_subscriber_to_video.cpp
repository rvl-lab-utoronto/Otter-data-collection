#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>

#include "navtech_msgs/msg/radar_configuration_msg.hpp"
#include "navtech_msgs/msg/radar_fft_data_msg.hpp"
#include "colossus_subscriber_to_video.h"
#include "net_conversion.h"

std::shared_ptr<Colossus_subscriber_to_video> node{};


Colossus_subscriber_to_video::Colossus_subscriber_to_video() :Node{ "colossus_subscriber_to_video" }
{
    using std::placeholders::_1;

    rclcpp::QoS qos_radar_configuration_subscriber(radar_configuration_queue_size);
    qos_radar_configuration_subscriber.reliable();

    configuration_data_subscriber =
    Node::create_subscription<navtech_msgs::msg::RadarConfigurationMsg>(
        "radar_data/configuration_data",
        qos_radar_configuration_subscriber,
        std::bind(&Colossus_subscriber_to_video::configuration_data_callback, this, _1)
    );

    rclcpp::QoS qos_radar_fft_subscriber(radar_fft_queue_size);
    qos_radar_fft_subscriber.reliable();

    fft_data_subscriber =
    Node::create_subscription<navtech_msgs::msg::RadarFftDataMsg>(
        "radar_data/fft_data",
        qos_radar_fft_subscriber,
        std::bind(&Colossus_subscriber_to_video::fft_data_callback, this, _1)
    );
}


void Colossus_subscriber_to_video::configuration_data_callback(const navtech_msgs::msg::RadarConfigurationMsg::SharedPtr msg) const
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
        RCLCPP_INFO(Node::get_logger(), "Bin Size: %f", msg->bin_size);
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

    if (!std::filesystem::exists(output_folder_name))
    {
        std::filesystem::create_directory(output_folder_name);
    }

    node->video_writer.open(output_folder_name + "/radar_output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), node->expected_rotation_rate / 1000, cv::Size(node->azimuth_samples, node->azimuth_samples), true);
    node->config_data_received = true;
}


void Colossus_subscriber_to_video::fft_data_callback(const navtech_msgs::msg::RadarFftDataMsg::SharedPtr msg) const
{
    if (!node->config_data_received) {
        RCLCPP_INFO(Node::get_logger(), "Configuration data not yet received");
        return;
    }


    if (msg->azimuth) {
        node->azimuth = msg->azimuth;
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Azimuth");
    }


    if (msg->data_length) {
        node->data_length = msg->data_length;
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Data Length");
    }

    node->current_bearing = (static_cast<double>(node->azimuth) / static_cast<double>(node->encoder_size)) * static_cast<double>(node->azimuth_samples);

    int max_index = std::min(static_cast<int>(node->data_length), static_cast<int>(node->azimuth_samples));
    int matrix_max_index = radar_image.rows * radar_image.cols * radar_image.channels();
    for (int i{ 0 }; i < max_index; i++) {
        int index = i * 1 + node->current_bearing * radar_image.step + 1;
        if (index < matrix_max_index) {
            image_ptr[index] = static_cast<int>(msg->data[i]);
        }
    }

    if (node->azimuth < node->last_azimuth) {
        cv::Mat recovered_lin_polar_img;
        cv::Point2f center{ static_cast<float>(radar_image.cols / 2), static_cast<float>(radar_image.rows / 2) };
        double max_radius = std::min(center.y, center.x);
        linearPolar(radar_image, recovered_lin_polar_img, center, max_radius, cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS + cv::WARP_INVERSE_MAP);
        cv::Mat normalised_image(cv::Size{ azimuth_samples, azimuth_samples }, CV_8UC1, cv::Scalar{ 0, 0 });
        normalize(recovered_lin_polar_img, normalised_image, 0, 255, cv::NORM_MINMAX);
        cv::Mat rotated_image(cv::Size{ azimuth_samples, azimuth_samples }, CV_8UC1, cv::Scalar{ 0, 0 });
        rotate(normalised_image, rotated_image, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::Mat channels[3] { blank_image, rotated_image, blank_image };
        cv::Mat merged_data;
        merge(channels, 3, merged_data);
        node->video_writer.write(merged_data);
    }
    node->last_azimuth = node->azimuth;
}