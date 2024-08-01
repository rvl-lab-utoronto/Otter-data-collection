#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "navtech_msgs/msg/radar_configuration_msg.hpp"
#include <vector>

#include "Colossus_TCP_client.h"
#include "Time_utils.h"
#include "Colossus_protocol.h"
#include "configurationdata.pb.h"
#include "Protobuf_helpers.h"
#include "Peak_finder.h"
#include "pointer_types.h"
#include "Units.h"
#include "Polar_coordinate.h"
#include "Cartesian_coordinate.h"

using Navtech::Networking::Colossus_protocol::TCP::Client;
using Navtech::Networking::Colossus_protocol::TCP::Message;

using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;
using namespace Navtech::Unit;
using namespace Navtech::Networking::Colossus_protocol::TCP;

using Navtech::Protobuf::from_vector_into;
using namespace Colossus;
using namespace Navtech::Navigation;
using namespace Navtech::Networking;


class Navigation_mode_point_cloud_publisher : public ::rclcpp::Node
{
public:
    Navigation_mode_point_cloud_publisher();
    ~Navigation_mode_point_cloud_publisher();

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
    constexpr static int radar_point_cloud_queue_size{ 4 };

    bool rotated_once(Azimuth_num azimuth);
    bool completed_full_rotation(Azimuth_num azimuth);

    // Owned components
    //
    Navtech::owner_of<Peak_finder> peak_finder { };
    Navtech::owner_of<Navtech::Networking::Colossus_protocol::TCP::Client> radar_client { };

    // Radar client callbacks
    //
    void configuration_data_handler(Client& radar_client [[maybe_unused]], Message& msg);
    void fft_data_handler(Client& radar_client [[maybe_unused]], Message& msg);
    void navigation_config_data_handler(Client& radar_client [[maybe_unused]], Message& msg);
    void navigation_data_handler(Client& radar_client [[maybe_unused]], Message& msg);
    std::vector<std::uint8_t> to_ros_format(const Degrees&, const Metre&, const Navtech::Unit::dB&);

    // Peak finder callback
    //
    void target_handler(const Navtech::Navigation::Azimuth_target& target_data);

    std::string radar_ip{ "" };
    uint16_t radar_port{ 0 };

    uint16_t start_azimuth{ 0 };
    uint16_t end_azimuth{ 0 };
    uint16_t start_bin{ 0 };
    uint16_t end_bin{ 0 };
    uint16_t azimuth_offset{ 0 };

    uint16_t bins_to_operate_on{ 0 };
    uint16_t min_bin{ 0 };
    double power_threshold{ 0 };
    uint32_t max_peaks_per_azimuth{ 0 };
    bool process_locally{ false };

    uint16_t points_this_rotation{ 0 };
    uint16_t max_possible_points{ 16000 };   // 400 azimuths, 10 points per azimuth max, 4 values per point
    std::vector<uint8_t> data_vector;

    int azimuth_samples{ 0 };
    int encoder_size{ 0 };
    float bin_size{ 0 };
    int range_in_bins{ 0 };
    int expected_rotation_rate{ 0 };
    int packet_rate{ 0 };
    float range_gain{ 0 };
    float range_offset{ 0 };
    int rotation_count{ 0 };
    int config_publish_count{ 4 };

    double combined_distance_offset{ 0.0 };
    double combined_distance_scale_factor{ 0.0 };
    double x_distance_offset{ 0.0 };
    double y_distance_offset{ 0.0 };

    void publish_point_cloud();
    void update_navigation_config();
    void update_local_navigation_config();
    void check_config_publish();

    navtech_msgs::msg::RadarConfigurationMsg config_message = navtech_msgs::msg::RadarConfigurationMsg{};
    Colossus_protocol::TCP::Configuration *config{};

    rclcpp::Publisher<navtech_msgs::msg::RadarConfigurationMsg>::SharedPtr configuration_data_publisher{};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher{};
};