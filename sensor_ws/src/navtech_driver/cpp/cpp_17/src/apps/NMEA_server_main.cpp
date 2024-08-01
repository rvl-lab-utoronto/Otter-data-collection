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
#include "NMEA_protocol.h"
#include "NMEA_server.h"
#include "Endpoint.h"
#include "Option_parser.h"
#include "Time_utils.h"
#include "Signal_handler.h"
#include "Log.h"

using namespace Navtech;

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;
using Navtech::Utility::Option_parser;
using Navtech::Utility::Option;
using Navtech::Utility::Signal_handler;

using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;

using Navtech::Networking::Endpoint;
using Navtech::Networking::IP_address;
using Navtech::Networking::Port;
using Navtech::Networking::NMEA_protocol::Server;
using Navtech::Networking::NMEA_protocol::Message;

// ---------------------------------------------------------------------------------------------------------------------
// Signal handling: If SIGINT or SIGTERM are sent to the 
// program, stop processing.
//
volatile bool running { true };

void stop_running(std::int32_t signal [[maybe_unused]], std::int32_t info [[maybe_unused]])
{
    stdout_log << "Ctrl-C received.  Terminating..." << endl;
    running = false;
}


// ---------------------------------------------------------------------------------------------------------------------
//
Option_parser options {
    "nmea_server",
    {
        Option { "--ipaddress", "-i", "Colossus server IP address", optional, has_argument, "127.0.0.1" },
        Option { "--port", "-p",      "Colossus server port",       optional, has_argument, "6317" },
    }
};


// ---------------------------------------------------------------------------------------------------------------------
//
int main(int argc, char** argv)
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
    auto addr = options["-i"].translate_to<IP_address>();
    auto port = options["-p"].to_int<std::uint16_t>();

    // Set the log output up with a simple time
    // format; and don't display debug messages
    //
    stdout_log.time_format("%T");
    stdout_log.min_level(Logging_level::info);
    

    Server server { Endpoint { addr, port } };

    // Message to send

    std::string message  { "$GPRMC,143535.300,A,5135.4604939,N,00122.9738391,W,2.337,104.604,240822,0.1,E,D*11\r\n" };

    server.start();

    while (running) {
        stdout_log << "Sending..." << endl;
        server.send(Message { message });
        sleep_for(5_sec);
    }

    server.stop();
    SDK::shutdown();
}
catch (std::exception& ex) {
    stdout_log << "TERMINATING MAIN DUE TO EXCEPTION: ";
    stdout_log << Logging_level::error << ex.what();
    stdout_log << endl;
    SDK::shutdown();
}
catch (...) {
    stdout_log << "TERMINATING MAIN DUE TO UNHANDLED EXCEPTION: ";
    SDK::shutdown();
}