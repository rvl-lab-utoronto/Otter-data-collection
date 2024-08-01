#include <rclcpp/qos.hpp>
#include "b_scan_extended_publisher.h"

#include "Endpoint.h"
#include "Time_utils.h"
#include "configurationdata.pb.h"

using Navtech::Networking::Colossus_protocol::TCP::Client;
using Navtech::Networking::Colossus_protocol::TCP::Message;

namespace Navtech {

using namespace Networking;

B_scan_extended_publisher::B_scan_extended_publisher() : Node { "b_scan_extended_publisher" }
{
    using namespace Networking::Colossus_protocol::TCP;

    declare_parameter("radar_ip", "");
    declare_parameter("radar_port", 0);
    declare_parameter("start_azimuth", 0);
    declare_parameter("end_azimuth", 0);
    declare_parameter("start_bin", 0);
    declare_parameter("end_bin", 0);
    declare_parameter("azimuth_offset", 0);

    radar_ip = get_parameter("radar_ip").as_string();
    RCLCPP_INFO(Node::get_logger(), "IP: %s", radar_ip.c_str());
    radar_port = get_parameter("radar_port").as_int();
    start_azimuth = get_parameter("start_azimuth").as_int();
    end_azimuth = get_parameter("end_azimuth").as_int();
    start_bin = get_parameter("start_bin").as_int();
    end_bin = get_parameter("end_bin").as_int();
    azimuth_offset = get_parameter("azimuth_offset").as_int();

    // Set up the radar client
    //
    Endpoint server_addr { Navtech::Networking::IP_address(radar_ip), radar_port };
    radar_client = Navtech::allocate_owned<Client>(
        server_addr
    );

    radar_client->set_handler(
        Type::configuration,
        std::bind(&B_scan_extended_publisher::configuration_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );

    radar_client->set_handler(
        Type::fft_data, 
        std::bind(&B_scan_extended_publisher::fft_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );

    radar_client->set_handler(
        Type::configuration, 
        std::bind(&B_scan_extended_publisher::configuration_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );

    rclcpp::QoS qos_radar_config_publisher(radar_configuration_queue_size);
    qos_radar_config_publisher.reliable();

    config_msg_publisher = 
        Node::create_publisher<navtech_msgs::msg::RadarConfigurationMsg>(
            "radar_data/configuration_data",
            qos_radar_config_publisher
    );

    rclcpp::QoS qos_b_scan_msg_publisher(b_scan_image_queue_size);
    qos_b_scan_msg_publisher.reliable();

    b_scan_msg_publisher =
        Node::create_publisher<navtech_msgs::msg::RadarBScanMsg>(
        "radar_data/b_scan_msg",
        qos_b_scan_msg_publisher
    );
}


B_scan_extended_publisher::~B_scan_extended_publisher()
{
    stop();
}


bool B_scan_extended_publisher::rotated_once(Unit::Azimuth_num azimuth)
{
    static bool has_rotated_once { };
    static Azimuth_num prev { 0 };
    
    if (has_rotated_once) return true;
    if (azimuth <= prev) has_rotated_once = true;
    prev = azimuth;

    return has_rotated_once;
}


bool B_scan_extended_publisher::completed_full_rotation(Unit::Azimuth_num azimuth)
{
    if (!rotated_once(azimuth)) return false;

    bool has_completed_rotation { false };
    static Azimuth_num prev { };

    if (azimuth <= prev) has_completed_rotation = true;
    prev = azimuth;

    return has_completed_rotation;
}


void B_scan_extended_publisher::start()
{
    radar_client->start();
}


void B_scan_extended_publisher::stop()
{
    using namespace Networking::Colossus_protocol::TCP;

    radar_client->remove_handler(Type::configuration);
    radar_client->remove_handler(Type::fft_data);
    
    radar_client->send(Type::stop_fft_data);
    radar_client->stop();
}


// Radar Callbacks
//
void B_scan_extended_publisher::configuration_data_handler(Client& radar_client [[maybe_unused]], const Message& msg) {
    using namespace Networking::Colossus_protocol::TCP;

    auto config   = msg.view_as<Configuration>();
    encoder_size = config->encoder_size();
    RCLCPP_INFO(Node::get_logger(), "Configuration Data Received");
    RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", config->azimuth_samples());
    RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", config->encoder_size());
    RCLCPP_INFO(Node::get_logger(), "Bin Size: %i", config->bin_size());
    RCLCPP_INFO(Node::get_logger(), "Range In Bins: : %i", config->range_in_bins());
    RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", config->rotation_speed());
    RCLCPP_INFO(Node::get_logger(), "Range Gain: %f", config->range_gain());
    RCLCPP_INFO(Node::get_logger(), "Range Offset: %f", config->range_offset());
    RCLCPP_INFO(Node::get_logger(), "Publishing Configuration Data");

    azimuth_samples     = config->azimuth_samples();
    encoder_size        = config->encoder_size();
    range_in_bins       = config->range_in_bins();
    bin_size            = config->bin_size();
    rotation_rate       = config->rotation_speed();
    range_gain          = config->range_gain();
    range_offset        = config->range_offset();

    buffer_length = azimuth_samples * range_in_bins * sizeof(uint8_t);
    
    encoder_values.resize(static_cast<size_t>(azimuth_samples));
    timestamps_seconds.resize(static_cast<size_t>(azimuth_samples));

    intensity_values.resize(static_cast<std::size_t>(azimuth_samples * range_in_bins));

    auto config_msg = navtech_msgs::msg::RadarConfigurationMsg();
    auto header = std_msgs::msg::Header();
    header.stamp = Node::get_clock()->now();
    header.frame_id = "navtech_config";
    
    config_msg.header = header;
    config_msg.azimuth_samples = azimuth_samples;
    config_msg.encoder_size = encoder_size;
    config_msg.bin_size = bin_size;
    config_msg.range_in_bins = range_in_bins;
    config_msg.expected_rotation_rate = rotation_rate;
    config_msg.range_gain = range_gain;
    config_msg.range_offset = range_offset;

    config_msg.azimuth_offset = azimuth_offset;

    config_msg_publisher->publish(config_msg);

    RCLCPP_INFO(Node::get_logger(), "Starting b scan publisher");
    RCLCPP_INFO(Node::get_logger(), "Start azimuth: %i", start_azimuth);
    RCLCPP_INFO(Node::get_logger(), "End azimuth: %i", end_azimuth);
    RCLCPP_INFO(Node::get_logger(), "Start bin: %i", start_bin);
    RCLCPP_INFO(Node::get_logger(), "End bin: %i", end_bin);
    RCLCPP_INFO(Node::get_logger(), "Azimuth offset: %i", azimuth_offset);

    has_config = true;

    radar_client.send(Type::start_fft_data);
}


void B_scan_extended_publisher::fft_data_handler(const Client& radar_client [[maybe_unused]], const Message& msg)
{
    using namespace Navtech::Networking;
    using namespace Navtech::Time::Monotonic;

    if (!has_config) return;
    
    auto fft  = msg.view_as<Colossus_protocol::TCP::FFT_data>();
    auto data = fft->to_vector();


    if (completed_full_rotation(fft->azimuth())) {
        rotation_count++;
        publish_b_scan();
        reinitialise_vectors();
    }

    int azimuth_index = (int)(fft->azimuth() / (encoder_size / azimuth_samples));

    // To adjust radar start azimuth, for sake of visualisation
    // Note - this value will be different for every setup!
    // Values based on 0 angle of radar, and surrounding landscape
    //
    int adjusted_azimuth_index = azimuth_index + azimuth_offset;
    if (adjusted_azimuth_index >= azimuth_samples) {
        adjusted_azimuth_index = adjusted_azimuth_index - azimuth_samples;
    }

    if ((azimuth_index >= start_azimuth) && (azimuth_index < end_azimuth)) {
        for (unsigned y = 0; y < data.size(); y++) {
            int adjusted_intensity_index = (adjusted_azimuth_index * range_in_bins) + y;
            if ((y >= start_bin) && (y < end_bin)) {
                intensity_values[adjusted_intensity_index] = data[y];

            }
            else {
                intensity_values[adjusted_intensity_index] = 0;
            }
        }
    }
    else {
        for (int y = 0; y < range_in_bins; y++) {
            intensity_values[(adjusted_azimuth_index * range_in_bins) + y] = 0;
        }
    }

    // Cache the encoder value and timestamps for the b_scan_msg
    //
    encoder_values[azimuth_index] = fft->azimuth();

    timestamps_seconds[azimuth_index] = fft->ntp_seconds()*1e9 + fft->ntp_split_seconds();

    if (azimuth_index == 199) {
        latest_scan_ts = fft->ntp_seconds();
        latest_scan_ts_ns = fft->ntp_split_seconds();
    }

 

    if (rotation_count < config_publish_count) return;

    int temp_azimuth_offset = get_parameter("azimuth_offset").as_int();
    if (temp_azimuth_offset > azimuth_samples){
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

    rotation_count = 0;
    
}


void B_scan_extended_publisher::publish_b_scan()
{
    if (intensity_values.size() != static_cast<std::size_t>(azimuth_samples * range_in_bins)) {
        return;
    }

    auto extended_msg = navtech_msgs::msg::RadarBScanMsg();
    auto b_scan = sensor_msgs::msg::Image();
    
    b_scan.header = std_msgs::msg::Header();
    b_scan.header.stamp.sec = latest_scan_ts;
    b_scan.header.stamp.nanosec = latest_scan_ts_ns;
    b_scan.header.frame_id = "b_scan_extended";

    b_scan.height = azimuth_samples;
    b_scan.width = range_in_bins;
    b_scan.encoding = "8UC1";
    b_scan.is_bigendian = false;
    b_scan.step = b_scan.width;
    b_scan.data = std::move(intensity_values);

    extended_msg.b_scan_img = b_scan;
    extended_msg.encoder_values = std::move(encoder_values);
    extended_msg.timestamps = std::move(timestamps_seconds);

    b_scan_msg_publisher->publish(extended_msg);
}


void B_scan_extended_publisher::reinitialise_vectors()
{
    encoder_values.resize(static_cast<std::size_t>(azimuth_samples));
    timestamps_seconds.resize(static_cast<std::size_t>(azimuth_samples));

    intensity_values.resize(static_cast<std::size_t>(azimuth_samples * range_in_bins));
}
} // namespace Navtech