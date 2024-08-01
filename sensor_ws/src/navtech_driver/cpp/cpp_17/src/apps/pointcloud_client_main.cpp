// ---------------------------------------------------------------------------------------------------------------------
// Copyright 2024 Navtech Radar Limited
// This file is part of IASDK which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//
// Disclaimer:
// Navtech Radar is furnishing this item "as is". Navtech Radar does not provide 
// any warranty of the item whatsoever, whether express, implied, or statutory,
// including, but not limited to, any warranty of merchantability or fitness
// for a particular purpose or any warranty that the contents of the item will
// be error-free.
// In no respect shall Navtech Radar incur any liability for any damages, including,
// but limited to, direct, indirect, special, or consequential damages arising
// out of, resulting from, or any way connected to the use of the item, whether
// or not based upon warranty, contract, tort, or otherwise; whether or not
// injury was sustained by persons or property or otherwise; and whether or not
// loss was sustained from, or arose out of, the results of, the item, or any
// services that may be provided by Navtech Radar.
// ---------------------------------------------------------------------------------------------------------------------
#include "sdk.h"
#include "Option_parser.h"
#include "Endpoint.h"
#include "Time_utils.h"
#include "Log.h"
#include "Signal_handler.h"

#include "Colossus_UDP_client.h"

using namespace Navtech;
using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;
using namespace Navtech::Networking;

using Navtech::Utility::Option_parser;
using Navtech::Utility::Option;

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;
using Navtech::Utility::linebreak;

using Navtech::Utility::Signal_handler;

// ---------------------------------------------------------------------------------------------------------------------
//
Option_parser options {
    "pointcloud_client",
    {
        Option { "--ipaddress", "-i", "IP address where UDP data arrives",  optional, has_argument, "127.0.0.1" },
        Option { "--port", "-p",      "Port to connect to",                 optional, has_argument, "6317" }
    }
};


// ---------------------------------------------------------------------------------------------------------------------
// Signal handling: If SIGINT or SIGTERM are sent to the 
// program, stop processing.
//
volatile bool running { true };

void stop_running(std::int32_t signal [[maybe_unused]], std::int32_t info [[maybe_unused]])
{
    running = false;
}

// ---------------------------------------------------------------------------------------------------------------------
// Message handling
//
void process_keep_alive(Colossus_protocol::UDP::Client&, Colossus_protocol::UDP::Message&)
{
    stdout_log << "Received keep alive!" << endl;
}


void process_point_cloud(Colossus_protocol::UDP::Client&, Colossus_protocol::UDP::Message& msg)
{

    auto spoke = msg.view_as<Colossus_protocol::UDP::Pointcloud_spoke>();

    stdout_log << linebreak;
    stdout_log << "Azimuth:  " << spoke->azimuth() << endl;
    stdout_log << "Bearing:  " << spoke->bearing().to_string() << endl;
    
    auto [sz, points] = spoke->points();

    stdout_log << "Points:   ";
    for (std::size_t i { 0 }; i < sz; ++i) {
        auto& pt = points[i];
        stdout_log << "[" << pt.range() << " m, " << pt.power() << " dB]";
    }
    stdout_log << endl;
}


// ---------------------------------------------------------------------------------------------------------------------
//
int main(int argc, char* argv[])
try
{
    SDK::initialise();

    // Set up signal handling for ctrl-c (SIGINT)
    // and kill (SIGTERM)
    // 
    Signal_handler signal_handler { };
    signal_handler.register_handler(SIGINT, stop_running);
    signal_handler.register_handler(SIGTERM, stop_running);
    
    // Command line option parsing
    //
    options.parse(argc, argv);
    auto recv_addr = options["-i"].translate_to<Networking::IP_address>();
    auto recv_port = options["-p"].translate_to<Networking::Port>();

    // Set the log output up with a simple time
    // format; and don't display debug messages
    //
    stdout_log.time_format("%T");
    stdout_log.min_level(Logging_level::info);
    stdout_log << "Starting..." << endl;

    Colossus_protocol::UDP::Client client { Endpoint { recv_addr, recv_port } };
    client.set_handler(Colossus_protocol::UDP::Type::keep_alive, process_keep_alive);
    client.set_handler(Colossus_protocol::UDP::Type::point_cloud, process_point_cloud);
    client.start();

    while (running) {
        sleep_for(500_msec);
    }

    stdout_log << "Stopping..." << endl;
    client.stop();
    SDK::shutdown();
    stdout_log << "Done." << endl;
}
catch (std::exception& e) {
    stdout_log << "Exception caught in main() : " << e.what() << endl;
    SDK::shutdown();
}
catch (...) {
    stdout_log << "UNHANDLED EXCEPTION!" << endl;
    SDK::shutdown();
}