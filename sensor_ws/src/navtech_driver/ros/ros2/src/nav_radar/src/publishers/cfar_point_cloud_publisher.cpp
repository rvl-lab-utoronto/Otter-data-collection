#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <math.h>

#include "navtech_msgs/msg/radar_configuration_msg.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "cfar_point_cloud_publisher.h"
#include "net_conversion.h"
#include "Endpoint.h"

#include "Colossus_TCP_client.h"
#include "Time_utils.h"
#include "Colossus_protocol.h"
#include "configurationdata.pb.h"
#include "Protobuf_helpers.h"

using Navtech::Networking::Colossus_protocol::TCP::Client;
using Navtech::Networking::Colossus_protocol::TCP::Message;

using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;
using namespace Navtech::Unit;
using namespace Navtech::Networking;
using namespace Navtech::Networking::Colossus_protocol::TCP;


namespace CFAR {

    class Window {
    public:
        Window(std::size_t window_size, std::uint32_t threshold_delta);

        std::size_t size() const;

        std::pair<std::uint8_t, std::size_t> process(const std::uint8_t* elem_ptr);
       
    private: 
        std::uint32_t delta { };
        std::size_t   window_sz { };
        std::size_t   threshold_exceeded_count { };

        static constexpr std::size_t guard_sz { 4 };  
    };


    Window::Window(std::size_t window_size, std::uint32_t threshold_delta) :
        delta       { threshold_delta },
        window_sz   { (window_size % 2) != 0 ? window_size : window_size + 1 }
    {
    }


    std::size_t Window::size() const
    {
        return window_sz;
    }


    std::pair<std::uint8_t, std::size_t> Window::process(const std::uint8_t* const elem_ptr)
    {
        const std::uint8_t* lower_begin { elem_ptr - (window_sz / 2) };
        const std::uint8_t* lower_end   { elem_ptr - guard_sz };
        const std::uint8_t* upper_begin { elem_ptr + guard_sz + 1 };
        const std::uint8_t* upper_end   { elem_ptr + (window_sz / 2) + 1 };

        auto lower_sum = std::accumulate(lower_begin, lower_end, 0);
        auto upper_sum = std::accumulate(upper_begin, upper_end, 0);
        auto elems     = (lower_end - lower_begin) + (upper_end - upper_begin);
        auto average   = (lower_sum + upper_sum) / elems;

        if (*elem_ptr > average + delta) {
            ++threshold_exceeded_count;
            return std::make_pair(*elem_ptr, threshold_exceeded_count);
        }
        else {
            return std::make_pair(0, threshold_exceeded_count);
        }
    }


    std::vector<std::uint8_t> cell_average(const std::vector<std::uint8_t>& azimuth, std::uint16_t window_size, std::uint16_t threshold_delta)
    {
        std::vector<std::uint8_t> output { };
        output.resize(azimuth.size());

        // Data output state data
        //
        std::uint16_t max_points { 2856 };

        Window window { window_size, threshold_delta };

        auto start = window.size() / 2;
        auto end   = azimuth.size() - ((window.size() / 2) + 1);

        for (auto i { start }; i < end; ++i) {
            auto [val, count] = window.process(&azimuth[i]);
            output[i] = val;
            if (count == max_points) break;
        }

        return output;
    }

} // namespace CFAR


bool Cfar_point_cloud_publisher::rotated_once(Azimuth_num azimuth)
{
    static bool has_rotated_once { };
    static Azimuth_num prev { 0 };
    
    if (has_rotated_once) return true;
    if (azimuth <= prev) has_rotated_once = true;
    prev = azimuth;

    return has_rotated_once;
}


bool Cfar_point_cloud_publisher::completed_full_rotation(Azimuth_num azimuth)
{
    if (!rotated_once(azimuth)) return false;

    bool has_completed_rotation { false };
    static Azimuth_num prev { };

    if (azimuth <= prev) has_completed_rotation = true;
    prev = azimuth;

    return has_completed_rotation;
}


Cfar_point_cloud_publisher::Cfar_point_cloud_publisher():Node{ "cfar_point_cloud_publisher" }
{
    declare_parameter("radar_ip", "");
    declare_parameter("radar_port", 0);
    declare_parameter("start_azimuth", 0);
    declare_parameter("end_azimuth", 0);
    declare_parameter("start_bin", 0);
    declare_parameter("end_bin", 0);
    declare_parameter("azimuth_offset", 0);
    declare_parameter("combined_distance_offset", 0.0);
    declare_parameter("combined_distance_scale_factor", 0.0);
    declare_parameter("x_distance_offset", 0.0);
    declare_parameter("y_distance_offset", 0.0);
    declare_parameter("window_size", 0);
    declare_parameter("threshold_delta", 0);

    radar_ip = get_parameter("radar_ip").as_string();
    radar_port = get_parameter("radar_port").as_int();
    start_azimuth = get_parameter("start_azimuth").as_int();
    end_azimuth = get_parameter("end_azimuth").as_int();
    start_bin = get_parameter("start_bin").as_int();
    end_bin = get_parameter("end_bin").as_int();
    azimuth_offset = get_parameter("azimuth_offset").as_int();
    combined_distance_offset = get_parameter("combined_distance_offset").as_double();
    combined_distance_scale_factor = get_parameter("combined_distance_scale_factor").as_double();
    x_distance_offset = get_parameter("x_distance_offset").as_double();
    y_distance_offset = get_parameter("y_distance_offset").as_double();
    window_size = get_parameter("window_size").as_int();
    threshold_delta = get_parameter("threshold_delta").as_int();

    // Set up the radar client
    //
    Endpoint server_addr { Navtech::Networking::IP_address(radar_ip), radar_port };
    radar_client = Navtech::allocate_owned<Navtech::Networking::Colossus_protocol::TCP::Client>(
        server_addr
    );


    radar_client->set_handler(
        Type::fft_data, 
        std::bind(&Cfar_point_cloud_publisher::fft_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );
    radar_client->set_handler(
        Type::configuration, 
        std::bind(&Cfar_point_cloud_publisher::configuration_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );

    rclcpp::QoS qos_radar_configuration_publisher(radar_configuration_queue_size);
    qos_radar_configuration_publisher.reliable();

    configuration_data_publisher =
    Node::create_publisher<navtech_msgs::msg::RadarConfigurationMsg>(
        "radar_data/configuration_data",
        qos_radar_configuration_publisher
    );

    rclcpp::QoS qos_cfar_point_cloud_publisher(radar_point_cloud_queue_size);
    qos_cfar_point_cloud_publisher.reliable();

    cfar_point_cloud_publisher =
    Node::create_publisher<sensor_msgs::msg::PointCloud2>(
        "radar_data/point_cloud",
        qos_cfar_point_cloud_publisher
    );

    data_vector.reserve(max_possible_points);
}


Cfar_point_cloud_publisher::~Cfar_point_cloud_publisher()
{
    stop();
}


void Cfar_point_cloud_publisher::start()
{
    radar_client->start();
}


void Cfar_point_cloud_publisher::stop()
{
    radar_client->remove_handler(Type::configuration);
    radar_client->remove_handler(Type::fft_data);

    radar_client->send(Type::stop_fft_data);
    radar_client->stop();
}


void Cfar_point_cloud_publisher::publish_point_cloud()
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

    cfar_point_cloud_publisher->publish(message);
}


void Cfar_point_cloud_publisher::fft_data_handler(Client& radar_client [[maybe_unused]], Message& msg)
{
    using namespace Navtech::Networking;
    using namespace Navtech::Time::Monotonic;
    using Navtech::Protobuf::from_vector_into;

    auto fft  = msg.view_as<Colossus_protocol::TCP::FFT_data>();
    auto data = fft->to_vector();

    int azimuth_index = (int)(fft->azimuth() / (encoder_size / azimuth_samples));

    // To adjust radar start azimuth, for sake of visualisation
    // Note - this value will be different for every setup!
    // Values based on 0 angle of radar, and surrounding landscape
    int adjusted_azimuth_index = azimuth_index + azimuth_offset;
    if (adjusted_azimuth_index >= azimuth_samples) {
        adjusted_azimuth_index = adjusted_azimuth_index - azimuth_samples;
    }


    // Run CFAR
    if ((azimuth_index >= start_azimuth) && (azimuth_index < end_azimuth)) {
        auto cfar_output = CFAR::cell_average(data, window_size, threshold_delta);
        for (unsigned bin_index = start_bin; bin_index < data.size(); bin_index++) {
            if ((bin_index >= start_bin) && (bin_index < end_bin)) {
                if (std::count(cfar_output.begin(), cfar_output.end(), bin_index)) {
                    auto bearing = static_cast<float>(adjusted_azimuth_index) / static_cast<float>(azimuth_samples) * 360.0;
                    auto range = ((bin_index * bin_size / 10000.0) * range_gain * combined_distance_scale_factor) + range_offset + combined_distance_offset;
                    auto vec = to_ros_format(bearing, range, data[bin_index]);
                    data_vector.insert(data_vector.end(), vec.begin(), vec.end());
                    points_this_rotation += 1;
                }
            }
        }
    }
    
    if (!completed_full_rotation(fft->azimuth())) {
        return;
    }

    rotation_count++;
    Cfar_point_cloud_publisher::publish_point_cloud();
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

        double temp_combined_distance_offset = get_parameter("combined_distance_offset").as_double();
        if (temp_combined_distance_offset < 0.0 || temp_combined_distance_offset > 1000000.0) {
            RCLCPP_INFO(Node::get_logger(), "Combined distance offset of %f is invalid, must be between %f and %f", temp_combined_distance_offset, 0.0, 1000000.0);
            RCLCPP_INFO(Node::get_logger(), "Setting combined distance offset to 0.0");
            set_parameter(rclcpp::Parameter("combined_distance_offset", 0.0));
        }
        else {
            combined_distance_offset = temp_combined_distance_offset;
        }

        double temp_combined_distance_scale_factor = get_parameter("combined_distance_scale_factor").as_double();
        if (temp_combined_distance_scale_factor < 0.0 || temp_combined_distance_scale_factor > 1000000.0) {
            RCLCPP_INFO(Node::get_logger(), "Combined distance scale factor of %f is invalid, must be between %f and %f", temp_combined_distance_scale_factor, 0.0, 1000000.0);
            RCLCPP_INFO(Node::get_logger(), "Setting combined distance scale factor to 0.0");
            set_parameter(rclcpp::Parameter("combined_distance_scale_factor", 0.0));
        }
        else {
            combined_distance_offset = temp_combined_distance_scale_factor;
        }

        double temp_x_distance_offset = get_parameter("x_distance_offset").as_double();
        if (temp_x_distance_offset < 0.0 || temp_x_distance_offset > 1000000.0) {
            RCLCPP_INFO(Node::get_logger(), "X distance offset of %f is invalid, must be between %f and %f", temp_x_distance_offset, 0.0, 1000000.0);
            RCLCPP_INFO(Node::get_logger(), "Setting X distance offset to 0.0");
            set_parameter(rclcpp::Parameter("x_distance_offset", 0.0));
        }
        else {
            x_distance_offset = temp_x_distance_offset;
        }

        double temp_y_distance_offset = get_parameter("y_distance_offset").as_double();
        if (temp_y_distance_offset < 0.0 || temp_y_distance_offset > 1000000.0) {
            RCLCPP_INFO(Node::get_logger(), "Y distance offset of %f is invalid, must be between %f and %f", temp_y_distance_offset, 0.0, 1000000.0);
            RCLCPP_INFO(Node::get_logger(), "Setting Y distance offset to 0.0");
            set_parameter(rclcpp::Parameter("y_distance_offset", 0.0));
        }
        else {
            y_distance_offset = temp_y_distance_offset;
        }

        int temp_window_size = get_parameter("window_size").as_int();
        if (temp_window_size < 0 || temp_window_size > 1000) {
            RCLCPP_INFO(Node::get_logger(), "Window size of %i is invalid, must be between 0 and %i", temp_window_size, 1000);
            RCLCPP_INFO(Node::get_logger(), "Setting window size to 0.0");
            set_parameter(rclcpp::Parameter("window_size", 0));
        }
        else {
            window_size = temp_window_size;
        }

        int temp_threshold_delta = get_parameter("threshold_delta").as_int();
        if (temp_threshold_delta < 0 || temp_threshold_delta > 255) {
            RCLCPP_INFO(Node::get_logger(), "Threshold delta of %i is invalid, must be between 0 and %i", temp_threshold_delta, 255);
            RCLCPP_INFO(Node::get_logger(), "Setting threshold delta to %i", 255);
            set_parameter(rclcpp::Parameter("threshold_delta", 255));
        }
        else {
            threshold_delta = temp_threshold_delta;
        }

        configuration_data_publisher->publish(config_message);
        rotation_count = 0;
    }
}


void Cfar_point_cloud_publisher::configuration_data_handler(Client& radar_client [[maybe_unused]], Message& msg)
{
    auto config   = msg.view_as<Configuration>();
    RCLCPP_INFO(Node::get_logger(), "Configuration Data Received");
    RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", config->azimuth_samples());
    RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", config->encoder_size());
    RCLCPP_INFO(Node::get_logger(), "Bin Size: %i", config->bin_size());
    RCLCPP_INFO(Node::get_logger(), "Range In Bins: : %i", config->range_in_bins());
    RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", config->rotation_speed());
    RCLCPP_INFO(Node::get_logger(), "Range Gain: %f", config->range_gain());
    RCLCPP_INFO(Node::get_logger(), "Range Offset: %f", config->range_offset());
    RCLCPP_INFO(Node::get_logger(), "Publishing Configuration Data");

    azimuth_samples = config->azimuth_samples();
    encoder_size = config->encoder_size();
    bin_size = config->bin_size();
    end_bin = config->range_in_bins();
    range_in_bins = config->range_in_bins();
    expected_rotation_rate = config->rotation_speed();
    range_gain = config->range_gain();
    range_offset = config->range_offset();
    config_message.header = std_msgs::msg::Header();
    config_message.header.stamp = get_clock()->now();
    config_message.header.frame_id = "point_cloud";
    config_message.azimuth_samples = config->azimuth_samples();
    config_message.encoder_size = config->encoder_size();
    config_message.bin_size = config->bin_size();
    config_message.range_in_bins = config->range_in_bins();
    config_message.expected_rotation_rate = config->rotation_speed();
    config_message.range_gain = config->range_gain();
    config_message.range_offset = config->range_offset();
    config_message.azimuth_offset = azimuth_offset;
    configuration_data_publisher->publish(config_message);

    RCLCPP_INFO(Node::get_logger(), "Starting point cloud publisher");
    RCLCPP_INFO(Node::get_logger(), "Start azimuth: %i", start_azimuth);
    RCLCPP_INFO(Node::get_logger(), "End azimuth: %i", end_azimuth);
    RCLCPP_INFO(Node::get_logger(), "Start bin: %i", start_bin);
    RCLCPP_INFO(Node::get_logger(), "End bin: %i", end_bin);
    RCLCPP_INFO(Node::get_logger(), "Azimuth offset: %i", azimuth_offset);
    RCLCPP_INFO(Node::get_logger(), "Combined distance offset: %f", combined_distance_offset);
    RCLCPP_INFO(Node::get_logger(), "Combined distance scale factor: %f", combined_distance_scale_factor);
    RCLCPP_INFO(Node::get_logger(), "X distance offset: %f", x_distance_offset);
    RCLCPP_INFO(Node::get_logger(), "Y distance offset: %f", y_distance_offset);
    RCLCPP_INFO(Node::get_logger(), "Windows size: %i", window_size);
    RCLCPP_INFO(Node::get_logger(), "Threshold delta: %i", threshold_delta);

    radar_client.send(Type::start_fft_data);
}


std::vector<std::uint8_t> Cfar_point_cloud_publisher::to_ros_format(const Degrees& bearing, const Metre& range, const Navtech::Unit::dB& power)
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