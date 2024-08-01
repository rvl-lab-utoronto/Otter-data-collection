#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <exception>
#include <cassert>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "navtech_msgs/msg/radar_configuration_msg.hpp"
#include "navtech_msgs/msg/radar_fft_data_msg.hpp"
#include "colossus_subscriber_laser_scan_publisher.h"
#include "net_conversion.h"
#include "Time_utils.h"
#include "configurationdata.pb.h"
#include "Protobuf_helpers.h"


bool Colossus_subscriber_laser_scan_publisher::rotated_once(int azimuth)
{
    static bool has_rotated_once { };
    static int prev { 0 };
    
    if (has_rotated_once) return true;
    if (azimuth <= prev) has_rotated_once = true;
    prev = azimuth;

    return has_rotated_once;
}


bool Colossus_subscriber_laser_scan_publisher::completed_full_rotation(int azimuth)
{
    if (!rotated_once(azimuth)) return false;

    bool has_completed_rotation { false };
    static int prev { };

    if (azimuth <= prev) has_completed_rotation = true;
    prev = azimuth;

    return has_completed_rotation;
}


Colossus_subscriber_laser_scan_publisher::Colossus_subscriber_laser_scan_publisher() : Node { "colossus_subscriber_laser_scan_publisher" }
{
    using std::placeholders::_1;

    declare_parameter("start_azimuth", 0);
    declare_parameter("end_azimuth", 0);
    declare_parameter("start_bin", 0);
    declare_parameter("end_bin", 0);
    declare_parameter("power_threshold", 0);
    declare_parameter("azimuth_offset", 0);
    declare_parameter("range_offset", 0.0);

    start_azimuth = get_parameter("start_azimuth").as_int();
    end_azimuth = get_parameter("end_azimuth").as_int();
    start_bin = get_parameter("start_bin").as_int();
    end_bin = get_parameter("end_bin").as_int();
    power_threshold = get_parameter("power_threshold").as_int();
    range_offset = get_parameter("range_offset").as_double();

    rclcpp::QoS qos_radar_configuration_subscriber(radar_configuration_queue_size);
    qos_radar_configuration_subscriber.reliable();

    configuration_data_subscriber = 
    Node::create_subscription<navtech_msgs::msg::RadarConfigurationMsg>(
        "radar_data/configuration_data",
        qos_radar_configuration_subscriber,
        std::bind(&Colossus_subscriber_laser_scan_publisher::configuration_data_callback, this, _1)
    );

    rclcpp::QoS qos_radar_fft_subscriber(radar_fft_queue_size);
    qos_radar_fft_subscriber.reliable();

    fft_data_subscriber =
    Node::create_subscription<navtech_msgs::msg::RadarFftDataMsg>(
        "radar_data/fft_data",
        qos_radar_fft_subscriber,
        std::bind(&Colossus_subscriber_laser_scan_publisher::fft_data_callback, this, _1)
    );

    rclcpp::QoS qos_laser_scan_publisher(radar_laser_scan_queue_size);
    qos_laser_scan_publisher.reliable();

    laser_scan_publisher =
    Node::create_publisher<sensor_msgs::msg::LaserScan>(
        "radar_data/laser_scan",
        qos_laser_scan_publisher
    );
}


void Colossus_subscriber_laser_scan_publisher::configuration_data_callback(const navtech_msgs::msg::RadarConfigurationMsg::SharedPtr msg)
{
    if (config_received)
    {
        return;
    }

    RCLCPP_INFO(Node::get_logger(), "Configuration Data recieved");


    if (msg->azimuth_samples) {
        azimuth_samples = msg->azimuth_samples;
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Azimuth Samples");
    }


    if (msg->encoder_size) {
        encoder_size = msg->encoder_size;
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Encoder Size");
    }


    if (msg->bin_size) {
        bin_size = msg->bin_size;
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Bin Size");
    }


    if (msg->range_in_bins) {
        end_bin = msg->range_in_bins;
        range_in_bins = msg->range_in_bins;
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: End Bin");
    }


    if (msg->expected_rotation_rate) {
        expected_rotation_rate = msg->expected_rotation_rate;
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Rotation Rate");
    }


    if (msg->range_gain) {
        range_gain = msg->range_gain;
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Range Gain");
    }


    if (msg->range_offset) {
        config_range_offset = msg->range_offset;
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Range Offset");
    }

    range_values.resize(azimuth_samples);
    intensity_values.resize(azimuth_samples);

    RCLCPP_INFO(Node::get_logger(), "Starting colossus subscriber laser scan publisher");
    RCLCPP_INFO(Node::get_logger(), "Start azimuth: %i", start_azimuth);
    RCLCPP_INFO(Node::get_logger(), "End azimuth: %i", end_azimuth);
    RCLCPP_INFO(Node::get_logger(), "Start bin: %i", start_bin);
    RCLCPP_INFO(Node::get_logger(), "End bin: %i", end_bin);
    RCLCPP_INFO(Node::get_logger(), "Power threshold: %i", power_threshold);
    RCLCPP_INFO(Node::get_logger(), "Azimuth offset: %i", azimuth_offset);
    RCLCPP_INFO(Node::get_logger(), "User range offset: %f", range_offset);
    RCLCPP_INFO(Node::get_logger(), "Range gain: %f", range_gain);
    RCLCPP_INFO(Node::get_logger(), "Range offset: %f", config_range_offset);

    config_received = true;
}


void Colossus_subscriber_laser_scan_publisher::fft_data_callback(const navtech_msgs::msg::RadarFftDataMsg::SharedPtr msg)
{
    using namespace Navtech::Networking;
    using namespace Navtech::Time::Monotonic;
    using Navtech::Protobuf::from_vector_into;

    if (config_received == false)
    {
        return;
    }

    auto data = msg->data;

    unsigned first_peak_bin_index;
    for (first_peak_bin_index = start_bin; first_peak_bin_index < std::min((unsigned int)data.size(), (unsigned int)end_bin); first_peak_bin_index++) {
        if (data[first_peak_bin_index] > power_threshold) {
            break;
        }
    }

    float range = (range_gain * ((bin_size * (first_peak_bin_index + 1) / 10000.0) + range_offset)) + config_range_offset;
    float intensity = data[first_peak_bin_index];
    
    int azimuth = 0;
    int azimuth_index = 0;

    if (msg->azimuth) {
        azimuth = msg->azimuth;
    }
    azimuth_index = (int)(azimuth / (encoder_size / azimuth_samples));

    // To adjust radar start azimuth, for sake of visualisation
    // Note - this value will be different for every setup!
    // Values based on 0 angle of radar, and surrounding landscape
    int adjusted_azimuth_index = azimuth_index + azimuth_offset;
    if (adjusted_azimuth_index >= azimuth_samples) {
        adjusted_azimuth_index = adjusted_azimuth_index - azimuth_samples;
    }

    if ((azimuth_index >= start_azimuth) && (azimuth_index < end_azimuth)) {
        range_values[adjusted_azimuth_index] = range;
        intensity_values[adjusted_azimuth_index] = intensity;
    }
    else{
        range_values[adjusted_azimuth_index] = 0;
        intensity_values[adjusted_azimuth_index] = 0;
    }

    if (!completed_full_rotation(azimuth)) {
        return;
    }

    rotation_count++;
    Colossus_subscriber_laser_scan_publisher::publish_laser_scan();

    if (rotation_count >= config_publish_count) {

        int temp_azimuth_offset = get_parameter("azimuth_offset").as_int();
        if (temp_azimuth_offset > azimuth_samples) {
            RCLCPP_INFO(Node::get_logger(), "Azimuth offset of %i is invalid, must be less than or equal to %i", temp_azimuth_offset, azimuth_samples);
            RCLCPP_INFO(Node::get_logger(), "Setting azimuth offset to %i", azimuth_samples);
            set_parameter(rclcpp::Parameter("azimuth_offset", azimuth_samples));
        }
        else {
            azimuth_offset = temp_azimuth_offset;
        }

        int temp_start_azimuth = get_parameter("start_azimuth").as_int();
        if (temp_start_azimuth < 0 || temp_start_azimuth > azimuth_samples) {
            RCLCPP_INFO(Node::get_logger(), "Start azimuth of %i is invalid, must be between 0 and %i", temp_start_azimuth, azimuth_samples);
            RCLCPP_INFO(Node::get_logger(), "Setting start azimuth to %i", 0);
            set_parameter(rclcpp::Parameter("start_azimuth", 0));
        }
        else {
            start_azimuth = temp_start_azimuth;
        }

        int temp_end_azimuth = get_parameter("end_azimuth").as_int();
        if (temp_end_azimuth < 0 || temp_end_azimuth > azimuth_samples) {
            RCLCPP_INFO(Node::get_logger(), "End azimuth of %i is invalid, must be between 0 and %i", temp_end_azimuth, azimuth_samples);
            RCLCPP_INFO(Node::get_logger(), "Setting end azimuth to %i", azimuth_samples);
            set_parameter(rclcpp::Parameter("end_azimuth", azimuth_samples));
        }
        else {
            end_azimuth = temp_end_azimuth;
        }

        int temp_start_bin = get_parameter("start_bin").as_int();
        if (temp_start_bin < 0 || temp_start_bin > range_in_bins) {
            RCLCPP_INFO(Node::get_logger(), "Start bin of %i is invalid, must be between 0 and %i", temp_start_bin, range_in_bins);
            RCLCPP_INFO(Node::get_logger(), "Setting start bin to %i", 0);
            set_parameter(rclcpp::Parameter("start_bin", 0));
        }
        else {
            start_bin = temp_start_bin;
        }

        int temp_end_bin = get_parameter("end_bin").as_int();
        if (temp_end_bin < 0 || temp_end_bin > range_in_bins) {
            RCLCPP_INFO(Node::get_logger(), "End bin of %i is invalid, must be between 0 and %i", temp_end_bin, range_in_bins);
            RCLCPP_INFO(Node::get_logger(), "Setting end bin to %i", range_in_bins);
            set_parameter(rclcpp::Parameter("end_bin", range_in_bins));
        }
        else {
            end_bin = temp_end_bin;
        }

        int temp_power_threshold = get_parameter("power_threshold").as_int();
        if (temp_power_threshold < 0 || temp_power_threshold > std::numeric_limits<uint8_t>::max()) {
            RCLCPP_INFO(Node::get_logger(), "Power threshold of %i is invalid, must be between 0 and %i", temp_power_threshold, std::numeric_limits<uint8_t>::max());
            RCLCPP_INFO(Node::get_logger(), "Setting power threshold to %i", std::numeric_limits<uint8_t>::max() / 2);
            set_parameter(rclcpp::Parameter("power_threshold", power_threshold));
        }
        else {
            power_threshold = temp_power_threshold;
        }

        double temp_range_offset = get_parameter("range_offset").as_double();
        if (temp_range_offset < 0.0 || temp_range_offset > ((bin_size / 10000) * range_in_bins)) {
            RCLCPP_INFO(Node::get_logger(), "Range offset of %f is invalid, must be between %f and %f", temp_range_offset, 0.0, ((bin_size / 10000) * range_in_bins));
            RCLCPP_INFO(Node::get_logger(), "Setting range offset to 0.0");
            set_parameter(rclcpp::Parameter("range_offset", 0.0));
        }
        else {
            range_offset = temp_range_offset;
        }

        rotation_count = 0;
    }
}


void Colossus_subscriber_laser_scan_publisher::publish_laser_scan()
{
    auto message = sensor_msgs::msg::LaserScan();

    message.header = std_msgs::msg::Header();
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "laser_frame";

    message.angle_min = M_PI / 180 * 360 / azimuth_samples * start_azimuth;
    message.angle_max = M_PI / 180 * 360 / azimuth_samples * end_azimuth;
    message.angle_increment = M_PI / 180 * 360 / azimuth_samples;
    message.time_increment = 1.0 / (expected_rotation_rate / 1000) / azimuth_samples;
    message.scan_time = 1.0 / expected_rotation_rate;
    message.range_min = bin_size / 10000.0;
    message.range_max = range_in_bins * bin_size / 10000.0;
    message.ranges.resize(azimuth_samples);
    message.ranges = range_values;
    message.intensities.resize(azimuth_samples);
    message.intensities = intensity_values;

    laser_scan_publisher->publish(message);
}