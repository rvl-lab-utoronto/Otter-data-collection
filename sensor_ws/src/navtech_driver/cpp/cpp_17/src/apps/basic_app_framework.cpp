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
// ---------------------------------------------------------------------------------------------------------------------
// This file is a basic app framework which can be used to construct your own
// applications.  It contains code for:
// - Parsing command line options
// - Signal handling
// - Configuring logging
// ---------------------------------------------------------------------------------------------------------------------
#include "sdk.h"
#include "Option_parser.h"
#include "Endpoint.h"
#include "Time_utils.h"
#include "Log.h"
#include "Signal_handler.h"

using namespace Navtech;
using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;

using Navtech::Utility::Option_parser;
using Navtech::Utility::Option;

using Navtech::Networking::IP_address;
using Navtech::Networking::Port;

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;

using Navtech::Utility::Signal_handler;

// ---------------------------------------------------------------------------------------------------------------------
//
Option_parser options {
    "colossus_client",
    {
        Option { "--ipaddress", "-i", "IP address to connect to", optional, has_argument, "127.0.0.1" },
        Option { "--port", "-p",      "Port to connect to",       optional, has_argument, "6317" },
        Option { "--listen", "-l",    "Server listen port",       optional, has_argument, "45911" }
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
//
int main(int argc, char* argv[])
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
    auto server_addr = options["-i"].translate_to<IP_address>();
    auto server_port = options["-p"].to_int<std::uint16_t>();
    auto listen_port = options["-l"].to_int<std::uint16_t>();

    stdout_log << "Starting..." << endl;

    while (running) {
        sleep_for(500_msec);
    }

    SDK::shutdown();
    stdout_log << "Done." << endl;
}