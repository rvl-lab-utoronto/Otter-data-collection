#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "navtech_msgs/msg/radar_configuration_msg.hpp"
#include "laser_scan_publisher.h"

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
using namespace Navtech::Networking::Colossus_protocol::TCP;


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Laser_scan_publisher>();

    RCLCPP_INFO(node->get_logger(), "Starting radar client");
    node->start();
    RCLCPP_INFO(node->get_logger(), "Radar client started");

    while (rclcpp::ok()) {
        spin(node);
    }

    RCLCPP_INFO(node->get_logger(), "Stopping radar client");
    node->stop();
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "Stopped radar client");
}