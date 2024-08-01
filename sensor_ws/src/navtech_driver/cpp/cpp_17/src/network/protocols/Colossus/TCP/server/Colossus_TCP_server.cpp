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

#include "Colossus_TCP_server.h"
#include "Log.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;


namespace Navtech::Networking::Colossus_protocol::TCP {

    Server::Server(const Port& listen_port) :
        server          { listen_port, Server_event::dispatcher },
        msg_dispatcher  { *this, Server_event::dispatcher }
    {
    }


    void Server::start()
    {
        if (running) return;

        stdout_log << "Colossus server - starting..." << endl;

        server.start();
        msg_dispatcher.start();

        running = true;
    }


    void Server::stop()
    {
        if (!running) return;

        stdout_log << "Colossus server - stopping..." << endl;

        running = false;

        msg_dispatcher.stop();
        msg_dispatcher.join();

        server.stop();

        stdout_log << "Colossus server - stopped" << endl;
    }


    void Server::set_handler(Type type, const Handler& handler)
    {
        msg_dispatcher.attach_to(type, handler);
    }


    void Server::remove_handler(Type type)
    {
        msg_dispatcher.detach_from(type);
    }


    void Server::send(const Message& msg)
    {
        server.send(msg);
    }


    void Server::send(Message&& msg)
    {
        server.send(std::move(msg));
    }


} // namespace Navtech::Networking::Colossus_protocol::TCP