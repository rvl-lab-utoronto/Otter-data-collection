#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "navtech_msgs/msg/radar_configuration_msg.hpp"
#include <vector>

#include "Colossus_UDP_client.h"
#include "Time_utils.h"
#include "Colossus_protocol.h"
#include "configurationdata.pb.h"
#include "Protobuf_helpers.h"
#include "Polar_coordinate.h"
#include "Cartesian_coordinate.h"
#include "net_conversion.h"
#include "Units.h"


using Navtech::Networking::Colossus_protocol::UDP::Client;
using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;
using namespace Navtech::Unit;
using namespace Navtech::Networking::Colossus_protocol::TCP;

class Native_point_cloud_publisher : public ::rclcpp::Node
{
public:
    Native_point_cloud_publisher();
    ~Native_point_cloud_publisher();

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
    constexpr static int radar_point_cloud_queue_size{ 4 };

    bool rotated_once(Azimuth_num azimuth);
    bool completed_full_rotation(Azimuth_num azimuth);

    // Owned components
    //
    Navtech::owner_of<Navtech::Networking::Colossus_protocol::UDP::Client> radar_client { };

    // Radar client callbacks
    //
    void point_cloud_data_handler(Client& radar_client [[maybe_unused]], Navtech::Networking::Colossus_protocol::UDP::Message&);
    std::vector<std::uint8_t> to_ros_format(const Degrees&, const Metre&, const Navtech::Unit::dB&);
    void send();

    std::string radar_ip{ "" };
    uint16_t radar_port{ 0 };
    
    uint16_t points_this_rotation{ 0 };
    uint16_t max_possible_points{ 16000 };   // 400 azimuths, 10 points per azimuth max, 4 values per point
    std::vector<uint8_t> data_vector;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr native_point_cloud_publisher{};
};