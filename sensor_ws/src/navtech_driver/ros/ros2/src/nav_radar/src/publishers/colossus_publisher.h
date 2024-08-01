#include <rclcpp/rclcpp.hpp>
#include "navtech_msgs/msg/radar_fft_data_msg.hpp"
#include "navtech_msgs/msg/radar_configuration_msg.hpp"

#include "Colossus_TCP_client.h"
#include "Time_utils.h"
#include "Colossus_protocol.h"
#include "configurationdata.pb.h"
#include "Units.h"

using Navtech::Networking::Colossus_protocol::TCP::Client;
using Navtech::Networking::Colossus_protocol::TCP::Message;

using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;
using namespace Navtech::Unit;
using namespace Navtech::Networking::Colossus_protocol::TCP;


class Colossus_publisher : public ::rclcpp::Node
{
public:
    Colossus_publisher();
    ~Colossus_publisher();

    void set_radar_ip(std::string ip) {
        radar_ip = ip;
    }

    std::string get_radar_ip() {
        return radar_ip;
    }

    void get_radar_port(uint16_t port) {
        radar_port = port;
    }

    uint16_t get_radar_port() {
        return radar_port;
    }

    void start();
    void stop();

private:
    constexpr static int radar_configuration_queue_size{ 1 };
    constexpr static int radar_fft_queue_size{ 400 };

    bool rotated_once(Azimuth_num azimuth);
    bool completed_full_rotation(Azimuth_num azimuth);

    // Owned components
    //
    Navtech::owner_of<Navtech::Networking::Colossus_protocol::TCP::Client> radar_client { };

    // Radar client callbacks
    //
    void configuration_data_handler(Client& radar_client [[maybe_unused]], Message& msg);
    void fft_data_handler(Client& radar_client [[maybe_unused]], Message& msg);
    void image_data_handler(Message& msg);

    std::string radar_ip{ "" };
    uint16_t radar_port{ 0 };

    int azimuth_samples{ 0 };
    int encoder_size{ 0 };
    int rotation_count{ 0 };
    int config_publish_count{ 4 };

    navtech_msgs::msg::RadarConfigurationMsg config_message = navtech_msgs::msg::RadarConfigurationMsg{};

    rclcpp::Publisher<navtech_msgs::msg::RadarConfigurationMsg>::SharedPtr configuration_data_publisher{};
    rclcpp::Publisher<navtech_msgs::msg::RadarFftDataMsg>::SharedPtr fft_data_publisher{};
};