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
#ifndef NMEA_SERVER_H
#define NMEA_SERVER_H

#include "Datagram_server.h"
#include "NMEA_protocol.h"

#include "Active.h"
#include "Endpoint.h"

namespace Navtech::Networking::NMEA_protocol {

    class Server : public Utility::Active {
    public:
        Server(const Endpoint& send_to);

        void send(const Message& msg);
        void send(Message&& msg);

    private:
        // Active class overrides
        //
        void on_start() override;
        void on_stop()  override;

        // Underlying server configuration
        //
        using Datagram_Server = Datagram_server<Protocol::nmea, Transport::udp>;
        
        // NOTE: Do not change this declaration order!
        //
        Endpoint        local_endpt;
        Endpoint        remote_endpt;
        Datagram_Server datagram_server;
       
        void on_send(Message& msg);

        // Helpers
        //
    };

} // namespace Navtech::Networking::NMEA_protocol


#endif // NMEA_SERVER_H