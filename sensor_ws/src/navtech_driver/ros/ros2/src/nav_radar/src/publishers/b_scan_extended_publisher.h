#ifndef B_SCAN_EXTENDED_PUBLISHER_H
#define B_SCAN_EXTENDED_PUBLISHER_H
#include <vector>
#include <memory>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include "navtech_msgs/msg/radar_b_scan_msg.hpp"
#include "navtech_msgs/msg/radar_configuration_msg.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include "Colossus_TCP_client.h"
#include "Colossus_protocol.h"
#include "Units.h"

using Navtech::Networking::Colossus_protocol::TCP::Client;
using Navtech::Networking::Colossus_protocol::TCP::Message;

namespace Navtech {

class B_scan_extended_publisher : public ::rclcpp::Node 
{
public:
    B_scan_extended_publisher();
    ~B_scan_extended_publisher();

    void start();
    void stop();

private:
    constexpr static int radar_configuration_queue_size { 1 };
    constexpr static int b_scan_image_queue_size { 4 };

    bool rotated_once(Unit::Azimuth azimuth);
    bool completed_full_rotation(Unit::Azimuth azimuth);

    // Owned components
    //
    owner_of<Networking::Colossus_protocol::TCP::Client> radar_client { };

    // Radar client callbacks
    //
    void configuration_data_handler(
        Networking::Colossus_protocol::TCP::Client& radar_client [[maybe_unused]],
        const Networking::Colossus_protocol::TCP::Message& msg
    );
    
    void fft_data_handler(
        const Networking::Colossus_protocol::TCP::Client& radar_client [[maybe_unused]],
        const Networking::Colossus_protocol::TCP::Message& msg
    );

    void publish_b_scan();
    void reinitialise_vectors();

    std::string radar_ip        { "" };
    uint16_t radar_port         { 0 };
    uint16_t start_azimuth      { 0 };
    uint16_t end_azimuth        { 0 };
    uint16_t start_bin          { 0 };
    uint16_t end_bin            { 0 };
    uint16_t azimuth_offset     { 0 };

    std::vector<std::uint8_t>   intensity_values    { };
    std::vector<Unit::Azimuth>  encoder_values      { };
    std::vector<uint64_t>          timestamps_seconds  { };

    std::uint32_t               latest_scan_ts      { };
    std::uint32_t               latest_scan_ts_ns   { };

    Unit::Azimuth       azimuth_samples     { 0 };
    Unit::Encoder_step  encoder_size        { 0 };
    Unit::Bin           range_in_bins       { 0 };
    Unit::Metre         bin_size            { 0.0f };
    Unit::mHz           rotation_rate       { 0 };
    Unit::Metre         range_gain          { 0.0f };
    Unit::Metre         range_offset        { 0.0f };

    int buffer_length           { 0 };
    int rotation_count          { 0 };
    int config_publish_count    { 4 };
    std::atomic_bool has_config { false };

    rclcpp::Publisher<navtech_msgs::msg::RadarBScanMsg>::SharedPtr b_scan_msg_publisher { };
    rclcpp::Publisher<navtech_msgs::msg::RadarConfigurationMsg>::SharedPtr config_msg_publisher { };
};

} // namespace Navtech
#endif