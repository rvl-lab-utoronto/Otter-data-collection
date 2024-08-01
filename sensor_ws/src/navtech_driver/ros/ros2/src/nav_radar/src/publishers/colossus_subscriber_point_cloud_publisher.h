#include <rclcpp/rclcpp.hpp>
#include "navtech_msgs/msg/radar_configuration_msg.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "Colossus_protocol.h"
#include <vector>
#include "Units.h"
#include "Polar_coordinate.h"
#include "Cartesian_coordinate.h"

using namespace Navtech::Unit;
using namespace Navtech::Networking::Colossus_protocol::TCP;


class Colossus_subscriber_point_cloud_publisher : public ::rclcpp::Node
{
public:
    Colossus_subscriber_point_cloud_publisher();

private:
    constexpr static int radar_configuration_queue_size{ 1 };
    constexpr static int radar_fft_queue_size{ 400 };
    constexpr static int radar_point_cloud_queue_size{ 4 };

    bool rotated_once(int azimuth);
    bool completed_full_rotation(int azimuth);

    void configuration_data_callback(const navtech_msgs::msg::RadarConfigurationMsg::SharedPtr msg);
    void fft_data_callback(const navtech_msgs::msg::RadarFftDataMsg::SharedPtr msg);
    std::vector<std::uint8_t> to_ros_format(const Degrees&, const Metre&, const Navtech::Unit::dB&);

    bool config_received { };
    uint16_t start_azimuth{ 0 };
    uint16_t end_azimuth{ 0 };
    uint16_t start_bin{ 0 };
    uint16_t end_bin{ 0 };
    uint16_t power_threshold{ 0 };
    uint16_t azimuth_offset{ 0 };

    double combined_distance_offset{ 0.0 };
    double combined_distance_scale_factor{ 0.0 };
    double x_distance_offset{ 0.0 };
    double y_distance_offset{ 0.0 };

    uint16_t points_this_rotation{ 0 };
    uint16_t max_possible_points{ 16000 };   // 400 azimuths, 10 points per azimuth max, 4 values per point
    std::vector<uint8_t> data_vector;

    int azimuth_samples{ 0 };
    int encoder_size{ 0 };
    float bin_size{ 0 };
    int range_in_bins{ 0 };
    int expected_rotation_rate{ 0 };
    double range_offset{ 0.0 };
    double range_gain{ 0.0 };
    int rotation_count{ 0 };
    int config_publish_count{ 4 };

    rclcpp::Subscription<navtech_msgs::msg::RadarConfigurationMsg>::SharedPtr configuration_data_subscriber;
    rclcpp::Subscription<navtech_msgs::msg::RadarFftDataMsg>::SharedPtr fft_data_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher{};

    void publish_point_cloud();
};