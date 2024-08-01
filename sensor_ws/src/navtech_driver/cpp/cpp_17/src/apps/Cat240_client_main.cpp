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

#include <cstdint>

#include "sdk.h"
#include "Cat240_client.h"
#include "Log.h"
#include "Option_parser.h"
#include "Time_utils.h"
#include "Cat240_protocol.h"
#include "Units.h"
#include "Statistical_value.h"
#include "Signal_handler.h"

using namespace Navtech;
using Navtech::Networking::Cat240_protocol::Client;
using Navtech::Networking::Cat240_protocol::Message;
using Navtech::Networking::Cat240_protocol::Type;
using Navtech::Networking::IP_address;
using Navtech::Networking::Port;
using Navtech::Networking::Endpoint;

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::pop_level;
using Navtech::Utility::Logging_level;
using Navtech::Utility::Option_parser;
using Navtech::Utility::Option;
using Navtech::Utility::Signal_handler;

using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;
using namespace Navtech::Unit;

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
Option_parser options {
    "cat240_client",
    {
        Option { "--ipaddress", "-i", "Server IP address", required, has_argument },
        Option { "--port", "-p",      "Server port",       required, has_argument }
    }
};


// ---------------------------------------------------------------------------------------------------------------------
//
bool rotated_once(Azimuth_num azimuth)
{
    static bool has_rotated_once { };
    static Azimuth_num prev { 0 };
    
    if (has_rotated_once) return true;
    if (azimuth < prev) has_rotated_once = true;
    prev = azimuth;

    return has_rotated_once;
}


bool completed_full_rotation(Azimuth_num azimuth)
{
    if (!rotated_once(azimuth)) return false;

    bool has_completed_rotation { false };
    static Azimuth_num prev { };

    if (azimuth < prev) has_completed_rotation = true;
    prev = azimuth;

    return has_completed_rotation;
}


void check_for_lost_packet(std::uint32_t counter, std::uint32_t packet_count)
{
    static bool first_update { true };
    static std::uint32_t prev { };

    if (first_update) {
        prev = counter;
        first_update = false;
        return;
    }

    if (counter != static_cast<std::uint32_t>(prev + 1)) {
        stdout_log << Logging_level::error << "Packets lost! "
                   << "packet [" << packet_count << "] "
                   << "current sweep counter [" << counter << "] "
                   << "previous [" << prev << "] "
                   << pop_level << endl;
    }

    prev = counter;
}


void process_FFT(Client& radar_client [[maybe_unused]], Message& msg)
{
    using namespace Navtech::Networking;
    using namespace Navtech::Time::Monotonic;
    using Utility::Statistical_value;

    static std::uint32_t packet_count { };
    static unsigned rotations { };
    static Observation t0 { now() };
    static Statistical_value<double, 10> packet_rate { };

    ++packet_count;

    auto fft  = msg.view_as<Cat240_protocol::Video>();

    // TODO - remove after debug
    //
    auto sweep_count = fft->sweep_counter();
    // stdout_log << "Message counter: " << sweep_count <<  ". Packet count: " << packet_count << endl;
    check_for_lost_packet(sweep_count, packet_count);

    auto data = fft->video_to_vector();
    // stdout_log << "Data size: " << data.size() << endl;

    auto time_of_day = Cat240_protocol::Time_of_day::overlay_at(fft->video_end());
    auto t_msg       = time_of_day->to_observation();
    // stdout_log << "Message time: " << t_msg.format_as("%T.ms").to_string() << endl;

    auto cfg = Cat240_protocol::Extended_info::overlay_at(time_of_day->end());
    // stdout_log << "Bin size:       " << cfg->bin_size() << endl;
    // stdout_log << "Range in bins:  " << cfg->range_in_bins() << endl;
    // stdout_log << "Encoder size:   " << cfg->encoder_size() << endl;
    // stdout_log << "Rotation speed: " << cfg->rotation_speed() << endl;
    // stdout_log << "Gain:           " << cfg->range_gain() << endl;
    // stdout_log << "Offset:         " << cfg->range_offset() << endl;

    auto azimuth = cfg->to_azimuth(fft->start_angle());
    // stdout_log << "Azimuth: " << azimuth << endl;

    if (!completed_full_rotation(azimuth)) return;

    ++rotations;
    auto t1 = now();
    auto rotation_period = t1 - t0;
    
    packet_rate = packet_count / rotation_period.to_sec();

    if (rotations % 10 == 0) {
        stdout_log  << "Rotation [" << rotations << "] "
                    << "FFT size [" << data.size() << "] "
                    << "average packet rate [" << packet_rate.mean() << "] "
                    << "Message time [" << t_msg.format_as("%T.ms").to_string() << "] "
                    << endl;

        // stdout_log << "Range in bins:  " << cfg->range_in_bins() << endl;
        // stdout_log << "Rotation speed: " << cfg->rotation_speed() << endl;
        // stdout_log << "Gain:           " << cfg->range_gain() << endl;
        // stdout_log << "Offset:         " << cfg->range_offset() << endl;
        // stdout_log << "Time:           " << cfg->time() << endl;
    }

    packet_count = 0;         
    t0 = t1;
}


// ---------------------------------------------------------------------------------------------------------------------
//
std::vector<std::uint8_t> to_buffer(const std::string& str)
{
    return std::vector<std::uint8_t> { str.begin(), str.end() };
}


// ---------------------------------------------------------------------------------------------------------------------
//
int main(int argc, char *argv[])
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

    Client client { Endpoint { addr, port } };
    client.set_handler(Type::video, process_FFT);
    client.start();
    
    while (running) {
        sleep_for(500_msec);
    }
    
    client.stop();
    SDK::shutdown();
}
catch (std::system_error& ex) {
    stdout_log << "EXCEPTION THROWN - " << ex.what() << endl;
    SDK::shutdown();
}