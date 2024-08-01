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
#include "Active.h"

#include "Colossus_TCP_client.h"
#include "Colossus_TCP_server.h"

using namespace Navtech;
using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;
using namespace Navtech::Networking;

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;
using Navtech::Utility::Option_parser;
using Navtech::Utility::Option;
using Navtech::Utility::Signal_handler;

// ---------------------------------------------------------------------------------------------------------------------
// The Relay class is a simple piece of code to demonstrate the use
// of the Colossus client and server.
// The Relay connects to a radar (server) and receives FFT data, which
// it forwards on via a client.
// Any commands coming in via the Relay's server are simply forwarded
// on the radar.
// The Relay performs no processing on the incoming FFT data.
//
class Relay : public Active {
public:
    Relay(const Endpoint& server, const Port& listen_port);

protected:
    void on_start() override;
    void on_stop()  override;

private:
    Endpoint server_addr;
    Port     listener_port;

    Colossus_protocol::TCP::Client client;
    Colossus_protocol::TCP::Server server;

    // Event handlers
    //
    Utility::Event_handler<Colossus_protocol::TCP::Message::ID> connect_handler { };
    Utility::Event_handler<Colossus_protocol::TCP::Message::ID> disconnect_handler { };

    void client_connected(const Colossus_protocol::TCP::Message::ID& id);
    void client_disconnected(const Colossus_protocol::TCP::Message::ID& id);

    // Colossus message handlers
    //
    void process_config(const Colossus_protocol::TCP::Message& msg);
    void process_FFT(const Colossus_protocol::TCP::Message& msg);
    void request_FFT(const Colossus_protocol::TCP::Message& msg);

    Colossus_protocol::TCP::Message config_msg  { };

    // Very simplistic client management
    //
    std::vector<Colossus_protocol::TCP::Message::ID> connected_clients { };
};


Relay::Relay(const Endpoint& server, const Port& listen_port) :
    client  { server },
    server  { listen_port }
{
}


void Relay::on_start()
{
    using namespace Colossus_protocol::TCP;

    stdout_log << "Relay starting..." << endl;

    // --------------------------------------------------------------------------------
    // Event handlers
    //
    connect_handler.when_notified_invoke(
        [this](const Message::ID& id) 
        { 
            async_call(&Relay::client_connected, this, id);
        }
    );
    Server_event::dispatcher.attach_to<Event::client_connected>(connect_handler);

    disconnect_handler.when_notified_invoke(
        [this](const Message::ID& id) 
        { 
            async_call(&Relay::client_disconnected, this, id);
        }
    );
    Server_event::dispatcher.attach_to<Event::client_disconnected>(disconnect_handler);

    // --------------------------------------------------------------------------------
    // Message handlers
    //
    client.set_handler(
        Type::configuration,
        [this](Client&, Message& msg) { process_config(msg); }
    );

    client.set_handler(
        Type::fft_data,
        [this](Client&, Message& msg) { process_FFT(msg); }
    );

    server.set_handler(
        Type::start_fft_data,
        [this](Server&, Message& msg) { request_FFT(msg); }
    );

    // Start the client, but *not* the server. Don't allow server connections 
    // until we've received a configuration from the radar
    //
    client.start();
}


void Relay::on_stop()
{
    using namespace Colossus_protocol::TCP;

    stdout_log << "Relay stopping..." << endl;

    server.stop();
    client.stop();

    Server_event::dispatcher.detach_from<Event::client_connected>(connect_handler);
    Server_event::dispatcher.detach_from<Event::client_disconnected>(disconnect_handler);
}


void Relay::client_connected(const Colossus_protocol::TCP::Message::ID& id)
{
    using namespace Colossus_protocol::TCP;

    stdout_log << "Relay - new client connection [" << id << "]" << endl;

    connected_clients.emplace_back(id);

    // When a client connects, the first action of the server must be
    // to send a configuration message. 
    // Don't forget to update the id for the newly-connected client.
    //
    config_msg.id(id);
    server.send(config_msg);
}


void Relay::client_disconnected(const Colossus_protocol::TCP::Message::ID& id)
{
    stdout_log << "Relay - disconnecting client [" << id << "]" << endl;

    connected_clients.erase(
        std::remove(connected_clients.begin(), connected_clients.end(), id),
        connected_clients.end()
    );
}


void Relay::process_config(const Colossus_protocol::TCP::Message& msg)
{
    using namespace Colossus_protocol::TCP;

    stdout_log << "Relay - Configuration received" << endl;

    // We don't do any processing on any of the messages so the config message 
    // is stored as-is so it can be sent out to any connecting clients when 
    // they connect.
    //
    config_msg = msg;

    // Now we have a configuration we can start up the server
    //
    server.start();
}


void Relay::request_FFT(const Colossus_protocol::TCP::Message& msg)
{
    using namespace Colossus_protocol::TCP;

    stdout_log << "Relay - client [" << msg.id() << "] requesting FFT data" << endl;

    client.send(Type::start_fft_data);
}


void Relay::process_FFT(const Colossus_protocol::TCP::Message& msg)
{
    using namespace Colossus_protocol::TCP;

    Message forwarded_msg { msg };

    // Simply forward on FFT data to any attached clients
    //
    for (auto id : connected_clients) {
        forwarded_msg.id(id);
        server.send(forwarded_msg);
    }
}


// ---------------------------------------------------------------------------------------------------------------------
//
Option_parser options {
    "tcp_relay",
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

    // Set the log output up with a simple time
    // format; and don't display debug messages
    //
    stdout_log.time_format("%T");
    stdout_log.min_level(Logging_level::info);
    
    // Command line option parsing
    //
    options.parse(argc, argv);
    auto server_addr = options["-i"].translate_to<IP_address>();
    auto server_port = options["-p"].to_int<Port>();
    auto listen_port = options["-l"].to_int<Port>();

    Relay relay { Endpoint { server_addr, server_port }, listen_port };

    stdout_log << "Starting..." << endl;

    relay.start();

    while (running) {
        sleep_for(500_msec);
    }

    relay.stop();
    SDK::shutdown();

    stdout_log << "Done." << endl;
}