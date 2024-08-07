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
#ifndef CAT240_SERVER_H
#define CAT240_SERVER_H

#include "Datagram_server.h"
#include "Cat240_events.h"
#include "Cat240_protocol.h"

#include "Active.h"

namespace Navtech::Networking::Cat240_protocol {

    class Server : public Utility::Active {
    public:
        Server();
        Server(const Endpoint& remote);
        Server(const Endpoint& remote, std::size_t max_packet_size);

        void send_to(const Endpoint& remote_addr);

        std::size_t max_packet_size() const;
        void max_packet_size(std::size_t sz);

        void send(const Message& msg);
        void send(Message&& msg);

    private:
        // Active class overrides
        //
        void on_start() override;
        void on_stop()  override;

        // Underlying server configuration
        //
        using Datagram_Server = Datagram_server<Protocol::cat240, Transport::udp>;
        
        Endpoint        local_endpt       { IP_address::any(), 6317 };
        std::size_t     max_packet_sz     { 8900 };
        Datagram_Server datagram_server;
       
        // Async Event handling
        //
        Utility::Event_handler<Message> msg_handler { };

        void on_received_msg(Message& addr_msg);
        void on_send(Cat240_protocol::Message& msg);

        // Helpers
        //
        std::uint32_t index { };
        std::uint32_t next_index();
        
        void send_as_single(Cat240_protocol::Message& msg);
        void send_as_multiple(Cat240_protocol::Message& msg);
        
        std::size_t max_payload_blocks(std::size_t payload_sz, std::size_t block_sz);
        std::size_t blocks_required(std::size_t payload_sz, std::size_t block_sz);
        std::size_t non_payload_size(const Cat240_protocol::Message& msg);
        std::tuple<std::size_t, std::size_t, std::size_t> payload_split_size(std::size_t req, std::size_t max, std::size_t blk_sz);
    };

} // namespace Navtech::Networking::Cat240_protocol


#endif // CAT240_SERVER_H