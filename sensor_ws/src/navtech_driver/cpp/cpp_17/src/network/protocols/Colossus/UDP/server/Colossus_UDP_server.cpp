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
#include "Colossus_UDP_server.h"


namespace Navtech::Networking::Colossus_protocol::UDP {

    Server::Server() :
        Active          { "Colossus UDP server" },
        datagram_server { local_endpt, Server_event::dispatcher }
    {
    }


    Server::Server(const Endpoint& remote_endpt) :
        Active          { "Colossus UDP server" },
        datagram_server { local_endpt, remote_endpt, Server_event::dispatcher }
    {
    }


    void Server::on_start()
    {
        datagram_server.start();
    }


    void Server::on_stop()
    {
        datagram_server.stop();
        datagram_server.join();
    }

   
    void Server::send_to(const Endpoint& remote)
    {
        datagram_server.send_to(remote);
    }


    void Server::send(const Message& msg)
    {
        async_call(&Server::on_send, this, msg);
    }


    void Server::send(Message&& msg)
    {
        async_call(&Server::on_send, this, std::move(msg));
    }


    void Server::on_send(Message& msg)
    {
        datagram_server.send(msg);
    }

} // namespace Navtech::Networking::Colossus_protocol::UDP