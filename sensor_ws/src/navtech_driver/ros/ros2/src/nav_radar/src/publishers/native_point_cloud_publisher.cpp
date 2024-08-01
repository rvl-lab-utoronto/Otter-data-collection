#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <math.h>

#include "navtech_msgs/msg/radar_configuration_msg.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "native_point_cloud_publisher.h"
#include "net_conversion.h"
#include "Endpoint.h"

#include "Colossus_UDP_client.h"
#include "Time_utils.h"
#include "Colossus_protocol.h"
#include "configurationdata.pb.h"
#include "Protobuf_helpers.h"

using Navtech::Networking::Colossus_protocol::UDP::Client;
using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;
using namespace Navtech::Unit;
using namespace Navtech::Networking;
using namespace Navtech::Networking::Colossus_protocol::TCP;


bool Native_point_cloud_publisher::rotated_once(Azimuth_num azimuth)
{
    static bool has_rotated_once { };
    static Azimuth_num prev { 0 };
    
    if (has_rotated_once) return true;
    if (azimuth <= prev) has_rotated_once = true;
    prev = azimuth;

    return has_rotated_once;
}


bool Native_point_cloud_publisher::completed_full_rotation(Azimuth_num azimuth)
{
    if (!rotated_once(azimuth)) return false;

    bool has_completed_rotation { false };
    static Azimuth_num prev { };

    if (azimuth <= prev) has_completed_rotation = true;
    prev = azimuth;

    return has_completed_rotation;
}


Native_point_cloud_publisher::Native_point_cloud_publisher():Node{ "native_point_cloud_publisher" }
{
    declare_parameter("radar_ip", "");
    declare_parameter("radar_port", 0);

    radar_ip = get_parameter("radar_ip").as_string();
    radar_port = get_parameter("radar_port").as_int();

    // Set up the radar client
    //
    Endpoint server_addr { Navtech::Networking::IP_address(radar_ip), radar_port };
    radar_client = Navtech::allocate_owned<Colossus_protocol::UDP::Client>(
        server_addr
    );

    radar_client->set_handler(
        Colossus_protocol::UDP::Type::point_cloud, 
        std::bind(&Native_point_cloud_publisher::point_cloud_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );

    rclcpp::QoS qos_native_point_cloud_publisher(radar_point_cloud_queue_size);
    qos_native_point_cloud_publisher.reliable();

    native_point_cloud_publisher =
    Node::create_publisher<sensor_msgs::msg::PointCloud2>(
        "radar_data/point_cloud",
        qos_native_point_cloud_publisher
    );

    data_vector.reserve(max_possible_points);
}


Native_point_cloud_publisher::~Native_point_cloud_publisher()
{
    stop();
}


void Native_point_cloud_publisher::start()
{
    radar_client->start();
}


void Native_point_cloud_publisher::stop()
{
    radar_client->remove_handler(Colossus_protocol::UDP::Type::point_cloud);

    radar_client->stop();
}


void Native_point_cloud_publisher::send()
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

    native_point_cloud_publisher->publish(message);
}


std::vector<std::uint8_t> Native_point_cloud_publisher::to_ros_format(const Degrees& bearing, const Metre& range, const Navtech::Unit::dB& power)
{
    using namespace std;

    vector<uint8_t> ros_field { };
    
    auto pos = Navtech::Polar::Coordinate(range, bearing).to_cartesian();
    auto x_vec = to_vector(pos.x);
    auto y_vec = to_vector(pos.y);
    auto z_vec = vector<uint8_t>(4);  // Always 0
    auto power_vec = to_vector(power);

    ros_field.insert(ros_field.end(), x_vec.begin(), x_vec.end());
    ros_field.insert(ros_field.end(), y_vec.begin(), y_vec.end());
    ros_field.insert(ros_field.end(), z_vec.begin(), z_vec.end());
    ros_field.insert(ros_field.end(), power_vec.begin(), power_vec.end());

    return ros_field;
}


void Native_point_cloud_publisher::point_cloud_data_handler(Client& radar_client [[maybe_unused]], Colossus_protocol::UDP::Message& msg)
{
    auto spoke = msg.view_as<Colossus_protocol::UDP::Pointcloud_spoke>();
    auto [sz, points] = spoke->points();

    for (unsigned i { 0 }; i < sz; ++i) {
        auto& pt = points[i];
        auto vec = to_ros_format(spoke->bearing(), pt.range(), pt.power());
        data_vector.insert(data_vector.end(), vec.begin(), vec.end());
    }
    points_this_rotation += sz;

    if (!completed_full_rotation(spoke->azimuth())) {
        return;
    }

    send();
    points_this_rotation = 0;
    data_vector.reserve(max_possible_points);
}