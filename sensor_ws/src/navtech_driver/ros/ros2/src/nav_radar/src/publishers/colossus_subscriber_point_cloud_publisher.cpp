#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <exception>
#include <cassert>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "navtech_msgs/msg/radar_configuration_msg.hpp"
#include "navtech_msgs/msg/radar_fft_data_msg.hpp"
#include "colossus_subscriber_point_cloud_publisher.h"
#include "net_conversion.h"
#include "Time_utils.h"
#include "configurationdata.pb.h"
#include "Protobuf_helpers.h"

using namespace Navtech::Networking;


bool Colossus_subscriber_point_cloud_publisher::rotated_once(int azimuth)
{
    static bool has_rotated_once { };
    static int prev { 0 };
    
    if (has_rotated_once) return true;
    if (azimuth <= prev) has_rotated_once = true;
    prev = azimuth;

    return has_rotated_once;
}


bool Colossus_subscriber_point_cloud_publisher::completed_full_rotation(int azimuth)
{
    if (!rotated_once(azimuth)) return false;

    bool has_completed_rotation { false };
    static int prev { };

    if (azimuth <= prev) has_completed_rotation = true;
    prev = azimuth;

    return has_completed_rotation;
}


Colossus_subscriber_point_cloud_publisher::Colossus_subscriber_point_cloud_publisher() : Node { "colossus_subscriber_point_cloud_publisher" }
{
    using std::placeholders::_1;

    declare_parameter("start_azimuth", 0);
    declare_parameter("end_azimuth", 0);
    declare_parameter("start_bin", 0);
    declare_parameter("end_bin", 0);
    declare_parameter("power_threshold", 0);
    declare_parameter("azimuth_offset", 0);
    declare_parameter("combined_distance_offset", 0.0);
    declare_parameter("combined_distance_scale_factor", 0.0);
    declare_parameter("x_distance_offset", 0.0);
    declare_parameter("y_distance_offset", 0.0);

    start_azimuth = get_parameter("start_azimuth").as_int();
    end_azimuth = get_parameter("end_azimuth").as_int();
    start_bin = get_parameter("start_bin").as_int();
    end_bin = get_parameter("end_bin").as_int();
    power_threshold = get_parameter("power_threshold").as_int();
    azimuth_offset = get_parameter("azimuth_offset").as_int();
    combined_distance_offset = get_parameter("combined_distance_offset").as_double();
    combined_distance_scale_factor = get_parameter("combined_distance_scale_factor").as_double();
    x_distance_offset = get_parameter("x_distance_offset").as_double();
    y_distance_offset = get_parameter("y_distance_offset").as_double();

    rclcpp::QoS qos_radar_configuration_subscriber(radar_configuration_queue_size);
    qos_radar_configuration_subscriber.reliable();

    configuration_data_subscriber = 
    Node::create_subscription<navtech_msgs::msg::RadarConfigurationMsg>(
        "radar_data/configuration_data",
        qos_radar_configuration_subscriber,
        std::bind(&Colossus_subscriber_point_cloud_publisher::configuration_data_callback, this, _1)
    );

    rclcpp::QoS qos_radar_fft_subscriber(radar_fft_queue_size);
    qos_radar_fft_subscriber.reliable();

    fft_data_subscriber =
    Node::create_subscription<navtech_msgs::msg::RadarFftDataMsg>(
        "radar_data/fft_data",
        qos_radar_fft_subscriber,
        std::bind(&Colossus_subscriber_point_cloud_publisher::fft_data_callback, this, _1)
    );

    rclcpp::QoS qos_point_cloud_publisher(radar_point_cloud_queue_size);
    qos_point_cloud_publisher.reliable();

    point_cloud_publisher =
    Node::create_publisher<sensor_msgs::msg::PointCloud2>(
        "radar_data/point_cloud",
        qos_point_cloud_publisher
    );

    data_vector.reserve(max_possible_points);
}


void Colossus_subscriber_point_cloud_publisher::configuration_data_callback(const navtech_msgs::msg::RadarConfigurationMsg::SharedPtr msg)
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


    if (msg->range_in_bins) {
        bin_size = msg->range_in_bins;
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
        range_offset = msg->range_offset;
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Range Offset");
    }

    RCLCPP_INFO(Node::get_logger(), "Starting colossus subscriber point cloud publisher");
    RCLCPP_INFO(Node::get_logger(), "Start azimuth: %i", start_azimuth);
    RCLCPP_INFO(Node::get_logger(), "End azimuth: %i", end_azimuth);
    RCLCPP_INFO(Node::get_logger(), "Start bin: %i", start_bin);
    RCLCPP_INFO(Node::get_logger(), "End bin: %i", end_bin);
    RCLCPP_INFO(Node::get_logger(), "Power threshold: %i", power_threshold);
    RCLCPP_INFO(Node::get_logger(), "Azimuth offset: %i", azimuth_offset);
    RCLCPP_INFO(Node::get_logger(), "Combined distance offset: %f", combined_distance_offset);
    RCLCPP_INFO(Node::get_logger(), "Combined distance scale factor: %f", combined_distance_scale_factor);
    RCLCPP_INFO(Node::get_logger(), "X distance offset: %f", x_distance_offset);
    RCLCPP_INFO(Node::get_logger(), "Y distance offset: %f", y_distance_offset);
    RCLCPP_INFO(Node::get_logger(), "Range gain: %f", range_gain);
    RCLCPP_INFO(Node::get_logger(), "Range offset: %f", range_offset);

    config_received = true;
}


void Colossus_subscriber_point_cloud_publisher::fft_data_callback(const navtech_msgs::msg::RadarFftDataMsg::SharedPtr msg)
{
    using namespace Navtech::Networking;
    using namespace Navtech::Time::Monotonic;
    using Navtech::Protobuf::from_vector_into;

    if (config_received == false)
    {
        return;
    }

    auto data = msg->data;
    
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
        for (unsigned bin_index = start_bin; bin_index < data.size(); bin_index++) {
            if ((bin_index >= start_bin) && (bin_index < end_bin)) {
                if (data[bin_index] > power_threshold) {
                    auto bearing = static_cast<float>(adjusted_azimuth_index) / static_cast<float>(azimuth_samples) * 360.0;
                    auto range = ((bin_index * bin_size / 10000.0) * range_gain * combined_distance_scale_factor) + range_offset + combined_distance_offset;
                    auto vec = to_ros_format(bearing, range, data[bin_index]);
                    data_vector.insert(data_vector.end(), vec.begin(), vec.end());
                    points_this_rotation += 1;
                }
            }
        }
    }

    if (!completed_full_rotation(azimuth)) {
        return;
    }

    rotation_count++;
    Colossus_subscriber_point_cloud_publisher::publish_point_cloud();
    data_vector.reserve(max_possible_points);
    points_this_rotation = 0;
    
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

        double temp_combined_distance_offset = get_parameter("combined_distance_offset").as_double();
        if (temp_combined_distance_offset < 0.0 - std::numeric_limits<double>::max() || temp_combined_distance_offset > std::numeric_limits<double>::max()) {
            RCLCPP_INFO(Node::get_logger(), "Combined distance offset of %f is invalid, must be between %f and %f", temp_combined_distance_offset, std::numeric_limits<double>::min(), std::numeric_limits<double>::max());
            RCLCPP_INFO(Node::get_logger(), "Setting combined distance offset to 0.0");
            set_parameter(rclcpp::Parameter("combined_distance_offset", 0.0));
        }
        else {
            combined_distance_offset = temp_combined_distance_offset;
        }

        double temp_combined_distance_scale_factor = get_parameter("combined_distance_scale_factor").as_double();
        if (temp_combined_distance_scale_factor < 0.0 - std::numeric_limits<double>::max() || temp_combined_distance_scale_factor > std::numeric_limits<double>::max()) {
            RCLCPP_INFO(Node::get_logger(), "Combined distance scale factor of %f is invalid, must be between %f and %f", temp_combined_distance_scale_factor, std::numeric_limits<double>::min(), std::numeric_limits<double>::max());
            RCLCPP_INFO(Node::get_logger(), "Setting combined distance scale factor to 0.0");
            set_parameter(rclcpp::Parameter("combined_distance_scale_factor", 0.0));
        }
        else {
            combined_distance_scale_factor = temp_combined_distance_scale_factor;
        }

        double temp_x_distance_offset = get_parameter("x_distance_offset").as_double();
        if (temp_x_distance_offset < 0.0 - std::numeric_limits<double>::max() || temp_x_distance_offset > std::numeric_limits<double>::max()) {
            RCLCPP_INFO(Node::get_logger(), "X distance offset of %f is invalid, must be between %f and %f", temp_x_distance_offset, std::numeric_limits<double>::min(), std::numeric_limits<double>::max());
            RCLCPP_INFO(Node::get_logger(), "Setting X distance offset to 0.0");
            set_parameter(rclcpp::Parameter("x_distance_offset", 0.0));
        }
        else {
            x_distance_offset = temp_x_distance_offset;
        }

        double temp_y_distance_offset = get_parameter("y_distance_offset").as_double();
        if (temp_y_distance_offset < 0.0 - std::numeric_limits<double>::max() || temp_y_distance_offset > std::numeric_limits<double>::max()) {
            RCLCPP_INFO(Node::get_logger(), "Y distance offset of %f is invalid, must be between %f and %f", temp_y_distance_offset, std::numeric_limits<double>::min(), std::numeric_limits<double>::max());
            RCLCPP_INFO(Node::get_logger(), "Setting Y distance offset to 0.0");
            set_parameter(rclcpp::Parameter("y_distance_offset", 0.0));
        }
        else {
            y_distance_offset = temp_y_distance_offset;
        }
    
        rotation_count = 0;
    }
}


void Colossus_subscriber_point_cloud_publisher::publish_point_cloud()
{
    auto message = sensor_msgs::msg::PointCloud2();
    message.header = std_msgs::msg::Header();
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "point_cloud";

    message.height = 1;
    message.width = points_this_rotation;
    const uint8_t data_type = 7;
    const uint8_t num_bytes = 4; //float32 as bytes

    auto x_field = sensor_msgs::msg::PointField();
    x_field.name = "x";
    x_field.offset = 0 * num_bytes;
    x_field.datatype = data_type;
    x_field.count = points_this_rotation;

    auto y_field = sensor_msgs::msg::PointField();
    y_field.name = "y";
    y_field.offset = 1 * num_bytes;
    y_field.datatype = data_type;
    y_field.count = points_this_rotation;

    auto z_field = sensor_msgs::msg::PointField();
    z_field.name = "z";
    z_field.offset = 2 * num_bytes;
    z_field.datatype = data_type;
    z_field.count = points_this_rotation;

    auto intensity_field = sensor_msgs::msg::PointField();
    intensity_field.name = "intensity";
    intensity_field.offset = 3 * num_bytes;
    intensity_field.datatype = data_type;
    intensity_field.count = points_this_rotation;

    message.fields = std::vector<sensor_msgs::msg::PointField>{x_field, y_field, z_field, intensity_field};

    message.is_bigendian = false;
    message.point_step = 4 * num_bytes;
    message.row_step = message.point_step * message.width;
    message.data = std::move(data_vector);
    message.is_dense = true;

    point_cloud_publisher->publish(message);
}


std::vector<std::uint8_t> Colossus_subscriber_point_cloud_publisher::to_ros_format(const Degrees& bearing, const Metre& range, const Navtech::Unit::dB& power)
{
    using namespace std;

    vector<uint8_t> ros_field { };
    
    auto pos = Navtech::Polar::Coordinate(range, bearing).to_cartesian();
    auto x_vec = to_vector(static_cast<float> (pos.x + x_distance_offset));
    auto y_vec = to_vector(static_cast<float> (pos.y + y_distance_offset));
    auto z_vec = vector<uint8_t>(4);  // Always 0
    auto power_vec = to_vector(power);

    ros_field.insert(ros_field.end(), x_vec.begin(), x_vec.end());
    ros_field.insert(ros_field.end(), y_vec.begin(), y_vec.end());
    ros_field.insert(ros_field.end(), z_vec.begin(), z_vec.end());
    ros_field.insert(ros_field.end(), power_vec.begin(), power_vec.end());

    return ros_field;
}