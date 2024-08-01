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
#include "Colossus_TCP_client.h"
#include "Message_writer.h"
#include "Protobuf_helpers.h"
#include "configurationdata.pb.h"

using namespace Navtech;
using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;

using Navtech::Utility::Option_parser;
using Navtech::Utility::Option;

using Navtech::Networking::IP_address;
using Navtech::Networking::Port;
using Navtech::Networking::Endpoint;

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;
using Navtech::Utility::Message_writer;
using Navtech::Utility::Signal_handler;

using namespace Navtech::Networking::Colossus_protocol;

// ---------------------------------------------------------------------------------------------------------------------
//
Option_parser options {
    "pointcloud_writer",
    {
        Option { "--ipaddress",     "-i",  "TCP address of radar",     optional, has_argument, "192.168.0.1" },
        Option { "--port",          "-p",  "TCP port of radar",        optional, has_argument, "6317" },
        Option { "--udpaddress",    "-u",  "UDP address for data",     optional, has_argument, "127.0.0.1" },
        Option { "--udpport",       "-d",  "UDP port for data",        optional, has_argument, "6317" },
        Option { "--filename",      "-f",  "File tag",                 optional, has_argument, "pointcloud" },
        Option { "--nometadata",    "-m",  "Turn off meta data",       optional, no_argument }
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
class Config_writer : public Message_writer<TCP::Client, TCP::Message> {
public:
    using Message_writer<TCP::Client, TCP::Message>::Message_writer;

protected:
    void on_write(const TCP::Message& msg) override;
    void on_callback(TCP::Client&, TCP::Message& msg);
};


void Config_writer::on_callback(TCP::Client& client, TCP::Message& msg)
{
    write(msg);
    client.send(TCP::Type::navigation_config_request);
}


void Config_writer::on_write(const TCP::Message& msg)
{
    using Navtech::Protobuf::from_vector_into;
    using Colossus::Protobuf::ConfigurationData;

    auto lens_str = [](Colossus::Protobuf::LensTypes lens)
                    {
                        static const char* strings[] {
                            "flat",
                            "cosecant",
                            "dish flat",
                            "dish cosecant",
                            "dish flat 4 x 2",
                            "dish flat 3 x 3",
                            "dish cosecant 4 x 2",
                            "dish cosecant 3 x 3",
                            "dish flat 2.8 x 3.6",
                            "dish cosecant 2.8 x 3.6"
                        };
                        return strings[static_cast<int>(lens)];
                    };

    auto orientation_str  = [](int type) { return type ? "hanging" : "normal"; };
    auto staring_mode_str = [](int mode) { return mode ? "enabled" : "disabled"; };

    stdout_log << "Writing radar configuration..." << endl;

    auto config   = msg.view_as<TCP::Configuration>();
    auto protobuf = from_vector_into<ConfigurationData>(config->to_vector()).value();
  
    output_file << "Radar configuration:" << std::endl;
    output_file << "====================" << std::endl;
    output_file << "Radar ID        [" << protobuf.model().id() << "]" << std::endl;
    output_file << "Model           [" << protobuf.model().name() << "]" << std::endl;
    output_file << "Unique ID       [" << protobuf.model().uniqueid() << "]" << std::endl;
    output_file << "Profile ID      [" << protobuf.model().profileid() << "]" << std::endl;
    output_file << std::endl;

    output_file << "OS              [" << protobuf.softwareversions().os() << "]" << std::endl;
    output_file << "Firmware        [" << protobuf.softwareversions().firmware() << "]" << std::endl;
    output_file << "UI              [" << protobuf.softwareversions().ui() << "]" << std::endl;
    output_file << "Radar FPGA      [" << protobuf.softwareversions().radarfpga() << "]" << std::endl;
    output_file << "Analogue FPGA   [" << protobuf.softwareversions().analogueboardfpga() << "]" << std::endl;
    output_file << "Digital FPGA    [" << protobuf.softwareversions().digitalboardfpga() << "]" << std::endl;
    output_file << std::endl;

    output_file << "Lens type       [" << lens_str(protobuf.nvramcontents().lenstype()) << "]" << std::endl;
    output_file << "Orientation     [" << orientation_str(protobuf.nvramcontents().orientation()) << "]" << std::endl;
    output_file << "Boss angle      [" << protobuf.nvramcontents().bossangle() << "]" << std::endl;
    output_file << std::endl;

    output_file << "Resolution (m)  [" << protobuf.rangeresolutionmetres() << "]" << std::endl;
    output_file << "Resolution (Hz) [" << protobuf.rangeresolutionhz() << "]" << std::endl;
    output_file << "Data width      [" << protobuf.datawidth() << "]" << std::endl;
    output_file << "Staring mode    [" << staring_mode_str(protobuf.staringmode()) << "]" << std::endl;
    output_file << std::endl;

    output_file << "Azimuth samples [" << config->azimuth_samples() << "]" << std::endl;
    output_file << "Range (bins)    [" << config->range_in_bins()<< "]" << std::endl;
    output_file << "Rotation (mHz)  [" << config->rotation_speed() << "]" << std::endl;
    output_file << "Range gain      [" << config->range_gain()<< "]" << std::endl;
    output_file << "Range offset    [" << config->range_offset()<< "]" << std::endl;
    output_file << std::endl;
}


// ---------------------------------------------------------------------------------------------------------------------
//
class Nav_config_writer : public Message_writer<TCP::Client, TCP::Message> {
public:
    using Message_writer<TCP::Client, TCP::Message>::Message_writer;

protected:
    void on_write(const TCP::Message& msg) override;
};


void Nav_config_writer::on_write(const TCP::Message& msg)
{
    stdout_log << "Writing navigation configuration..." << endl;

    auto hdr = msg.view_as<TCP::Navigation_config>();

    output_file << "Navigation configuration:" << std::endl;
    output_file << "=========================" << std::endl;
    output_file << "Bins to operate on    [" << hdr->bins_to_operate_on() << "]" << std::endl;
    output_file << "Min bin to operate on [" << hdr->min_bin_to_operate_on() << "]" << std::endl;
    output_file << "Threshold             [" << hdr->navigation_threshold() << "]" << std::endl;
    output_file << "Max peaks per azimuth [" << hdr->max_peaks_per_azimuth() << "]" << std::endl;
    output_file << std::endl;
}


// ---------------------------------------------------------------------------------------------------------------------
//
class Pointcloud_writer : public Message_writer<UDP::Client, UDP::Message> {
public:
    using Message_writer<UDP::Client, UDP::Message>::Message_writer;

protected:
    void on_write(const UDP::Message& msg) override;
};



void Pointcloud_writer::on_write(const UDP::Message& msg)
{
    auto spoke = msg.view_as<UDP::Pointcloud_spoke>();

    output_file << spoke->seconds() << ", ";
    output_file << spoke->split_seconds() << ", ";
    output_file << spoke->azimuth() << ", ";
    output_file << spoke->bearing().to_float() << ", ";

    auto [sz, points] = spoke->points();

    for (std::size_t i { 0 }; i < sz; ++i) {
        auto& pt = points[i];
        if (i != 0) output_file << ", ";
        output_file << pt.range() << ", " << pt.power();
    }

    output_file << std::endl;
}

// ---------------------------------------------------------------------------------------------------------------------
//
std::pair<std::string, std::string> output_filenames(const std::string& output_file)
{
    auto date_prefix = Real_time::Clock::now().format_as("%Y%m%d_%H%M%S").to_string();
    std::string filename { date_prefix + "_" + output_file };
    std::string data_file { filename + ".csv" };
    std::string meta_data { filename + ".cfg" };

    return { std::move(data_file), std::move(meta_data) };
}


void create_config_data(const Endpoint& server, std::string_view config_file)
{
    stdout_log << "Writing configuration data to " << config_file << endl;

    TCP::Client       tcp_client     { server };
    Config_writer     cfg_writer     { config_file, std::ios_base::trunc };
    Nav_config_writer nav_cfg_writer { config_file, std::ios_base::app };

    tcp_client.set_handler(TCP::Type::configuration, cfg_writer());
    tcp_client.set_handler(TCP::Type::navigation_configuration, nav_cfg_writer());

    cfg_writer.start();
    nav_cfg_writer.start();
    tcp_client.start();

    sleep_for(1_sec);
    
    tcp_client.stop();
    nav_cfg_writer.stop();
    cfg_writer.stop();
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

    // Set the log output up with a simple time
    // format; and don't display debug messages
    //
    stdout_log.time_format("%T");
    stdout_log.min_level(Logging_level::info);
    
    // Command line option parsing
    //
    options.parse(argc, argv);
    auto tcp_addr   = options["-i"].translate_to<IP_address>();
    auto tcp_port   = options["-p"].to_int<std::uint16_t>();
    auto udp_addr   = options["-u"].translate_to<IP_address>();
    auto udp_port   = options["-d"].to_int<std::uint16_t>();
    auto file_tag   = options["-f"].value();

    auto output_mode        { std::ios_base::out };
    Endpoint server_endpt   { tcp_addr, tcp_port };
    Endpoint data_endpt     { udp_addr, udp_port };

    stdout_log << "Starting..." << endl;

    auto [data_file, config_file] = output_filenames(file_tag);

    // Unless the user has disabled it from the command-line, 
    // get the configuration from the radar
    //
    if (!options["-m"]) {
        output_mode = std::ios_base::app;
        create_config_data(server_endpt, config_file);
    }

    stdout_log << "Writing point-cloud data to " << data_file << endl;

    UDP::Client       udp_client { data_endpt };
    Pointcloud_writer writer     { data_file, output_mode };
    
    udp_client.set_handler(UDP::Type::point_cloud, writer());

    writer.start();
    udp_client.start();

    while (running) {
        sleep_for(500_msec);
    }

    udp_client.stop();
    writer.stop();
    SDK::shutdown();

    stdout_log << "Done." << endl;
}