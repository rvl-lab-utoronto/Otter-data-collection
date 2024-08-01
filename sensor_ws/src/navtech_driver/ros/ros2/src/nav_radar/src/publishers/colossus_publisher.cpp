#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "navtech_msgs/msg/radar_configuration_msg.hpp"
#include "navtech_msgs/msg/radar_fft_data_msg.hpp"
#include "navtech_msgs/msg/radar_configuration_msg.hpp"
#include "colossus_publisher.h"
#include "net_conversion.h"
#include "Endpoint.h"

#include "Colossus_TCP_client.h"
#include "Time_utils.h"
#include "Colossus_protocol.h"
#include "configurationdata.pb.h"
#include "Protobuf_helpers.h"

using namespace Navtech::Networking;
using Navtech::Networking::Colossus_protocol::TCP::Client;
using Navtech::Networking::Colossus_protocol::TCP::Message;

using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;
using namespace Navtech::Unit;
using namespace Navtech::Networking::Colossus_protocol::TCP;


bool Colossus_publisher::rotated_once(Azimuth_num azimuth)
{
    static bool has_rotated_once { };
    static Azimuth_num prev { 0 };
    
    if (has_rotated_once) return true;
    if (azimuth <= prev) has_rotated_once = true;
    prev = azimuth;

    return has_rotated_once;
}


bool Colossus_publisher::completed_full_rotation(Azimuth_num azimuth)
{
    if (!rotated_once(azimuth)) return false;

    bool has_completed_rotation { false };
    static Azimuth_num prev { };

    if (azimuth <= prev) has_completed_rotation = true;
    prev = azimuth;

    return has_completed_rotation;
}


Colossus_publisher::Colossus_publisher():Node{ "colossus_publisher" }
{
    declare_parameter("radar_ip", "");
    declare_parameter("radar_port", 0);

    radar_ip = get_parameter("radar_ip").as_string();
    radar_port = get_parameter("radar_port").as_int();

    // Set up the radar client
    //
    Endpoint server_addr { Navtech::Networking::IP_address(radar_ip), radar_port };
    radar_client = Navtech::allocate_owned<Navtech::Networking::Colossus_protocol::TCP::Client>(
        server_addr
    );

    radar_client->set_handler(
        Type::fft_data, 
        std::bind(&Colossus_publisher::fft_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );
    radar_client->set_handler(
        Type::configuration, 
        std::bind(&Colossus_publisher::configuration_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );

    rclcpp::QoS qos_radar_configuration_publisher(radar_configuration_queue_size);
    qos_radar_configuration_publisher.reliable();

    configuration_data_publisher =
    Node::create_publisher<navtech_msgs::msg::RadarConfigurationMsg>(
        "radar_data/configuration_data",
        qos_radar_configuration_publisher
    );

    rclcpp::QoS qos_radar_fft_publisher(radar_fft_queue_size);
    qos_radar_fft_publisher.reliable();

    fft_data_publisher =
    Node::create_publisher<navtech_msgs::msg::RadarFftDataMsg>(
        "radar_data/fft_data",
        qos_radar_fft_publisher
    );
}


Colossus_publisher::~Colossus_publisher()
{
    stop();
}


void Colossus_publisher::start()
{
    radar_client->start();
}


void Colossus_publisher::stop()
{
    radar_client->remove_handler(Type::configuration);
    radar_client->remove_handler(Type::fft_data);

    radar_client->send(Type::stop_fft_data);
    radar_client->stop();
}


void Colossus_publisher::fft_data_handler(Client& radar_client [[maybe_unused]], Message& msg)
{
    using namespace Navtech::Networking;
    using namespace Navtech::Time::Monotonic;

    auto fft  = msg.view_as<Colossus_protocol::TCP::FFT_data>();
    auto data = fft->to_vector();

    auto message = navtech_msgs::msg::RadarFftDataMsg();
    message.header = std_msgs::msg::Header();
    message.header.stamp = Node::get_clock()->now();
    message.angle = fft->azimuth() / azimuth_samples * 360.0;
    message.azimuth = fft->azimuth();
    message.sweep_counter = fft->sweep_counter();
    message.ntp_seconds = fft->ntp_seconds();
    message.ntp_split_seconds = fft->ntp_split_seconds();
    message.data = data;
    message.data_length = data.size();

    fft_data_publisher->publish(message);

    if (!completed_full_rotation(fft->azimuth())) {
        return;
    }

    rotation_count++;

    if (rotation_count >= config_publish_count) {
        configuration_data_publisher->publish(config_message);
        rotation_count = 0;
    }
}


void Colossus_publisher::configuration_data_handler(Client& radar_client [[maybe_unused]], Message& msg)
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
    config_message.header = std_msgs::msg::Header();
    config_message.header.stamp = Node::get_clock()->now();
    config_message.azimuth_samples = config->azimuth_samples();
    config_message.encoder_size = config->encoder_size();
    config_message.bin_size = config->bin_size();
    config_message.range_in_bins = config->range_in_bins();
    config_message.expected_rotation_rate = config->rotation_speed();
    config_message.range_gain = config->range_gain();
    config_message.range_offset = config->range_offset();
    configuration_data_publisher->publish(config_message);

    // We only want to publish non contoured data
    radar_client.send(Type::start_non_contour_fft_data);
}