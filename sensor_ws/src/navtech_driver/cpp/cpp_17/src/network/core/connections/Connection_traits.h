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
#ifndef CONNECTION_TRAITS_H
#define CONNECTION_TRAITS_H

#include <vector>
#include <cstdint>

#include "Colossus_protocol_traits.h"
#include "CP_protocol_traits.h"
#include "Cat240_protocol_traits.h"
#include "NMEA_protocol_traits.h"
#include "TLS_traits.h"
#include "transport_traits.h"

#include "TCP_socket.h"
#include "UDP_socket.h"

#include "Memory_types.h"


namespace Navtech::Networking {

    template <Protocol, Transport, TLS::Type>
    class Connection_traits { };


    // ----------------------------------------------------------------------------------------------------------------
    //
    template <>
    class Connection_traits<Protocol::colossus, Transport::tcp, TLS::Type::none> {
    public:
        // Types
        //
        using Socket            = TCP_socket;
        using Message_buffer    = std::vector<std::uint8_t>;

        using Protocol_traits   = Navtech::Networking::Protocol_traits<Protocol::colossus, Transport::tcp, TLS::Type::none>;
        using Message           = typename Protocol_traits::Message;
        using ID                = typename Protocol_traits::ID;
   
        // Constants
        //
        static constexpr std::size_t max_buffer_size { 4196 };

        // Functions
        //
        static void set_socket_options(Socket& socket)
        {
            using namespace Navtech::Unit;

            socket.tx_buffer(2_kB);
            socket.tx_timeout(2_sec);

            // For a more detailed explanation behind these settings, visit:
            // https://blog.cloudflare.com/when-tcp-sockets-refuse-to-die/
            //
            socket.keep_alive(true);
            socket.keep_alive_idle(5_sec);
            socket.keep_alive_probes(5);
            socket.keep_alive_interval(1_sec);
            socket.user_timeout(7_sec); 

            socket.no_delay(true);
        }
    };


    // ----------------------------------------------------------------------------------------------------------------
    //
    template <>
    class Connection_traits<Protocol::colossus, Transport::udp, TLS::Type::none> {
    public:
        // Types
        //
        using Socket            = UDP_socket;
        using Message_buffer    = std::vector<std::uint8_t>;

        using Protocol_traits   = Navtech::Networking::Protocol_traits<Protocol::colossus, Transport::udp, TLS::Type::none>;
        using Message           = typename Protocol_traits::Message;
        using ID                = typename Protocol_traits::ID;
   
        // Constants
        //
        static constexpr std::size_t max_buffer_size { 4196 };

        // Functions
        //
        static void set_socket_options(Socket& socket)
        {
            // TODO - 
        }
    };


    // ----------------------------------------------------------------------------------------------------------------
    //
    template <>
    class Connection_traits<Protocol::cp, Transport::tcp, TLS::Type::none> {
    public:
        // Types
        //
        using Socket            = TCP_socket;
        using Message_buffer    = std::vector<std::uint8_t>;

        using Protocol_traits   = Navtech::Networking::Protocol_traits<Protocol::cp, Transport::tcp, TLS::Type::none>;
        using Message           = typename Protocol_traits::Message;
        using ID                = typename Protocol_traits::ID;
   
        // Constants
        //
        static constexpr Unit::Kilobyte max_buffer_size { 4_kB };

        // Functions
        //
        static void set_socket_options(Socket& socket)
        {
            using namespace Navtech::Unit;

            socket.tx_buffer(0_kB);
            socket.tx_timeout(2_sec);

            // For a more detailed explanation behind these settings, visit:
            // https://blog.cloudflare.com/when-tcp-sockets-refuse-to-die/
            //
            socket.keep_alive(true);
            socket.keep_alive_idle(5_sec);
            socket.keep_alive_probes(5);
            socket.keep_alive_interval(1_sec);
            socket.user_timeout(7_sec); 
        }
    };


    // ----------------------------------------------------------------------------------------------------------------
    //
    template <>
    class Connection_traits<Protocol::cp, Transport::tcp, TLS::Type::botan> {
    public:
        // Types
        //
        using Socket            = TCP_socket;
        using Message_buffer    = std::vector<std::uint8_t>;

        using Protocol_traits   = Navtech::Networking::Protocol_traits<Protocol::cp, Transport::tcp, TLS::Type::botan>;
        using Message           = typename Protocol_traits::Message;
        using ID                = typename Protocol_traits::ID;
   
        // Constants
        //
        static constexpr Unit::Kilobyte max_buffer_size { 4_kB };

        // Functions
        //
        static void set_socket_options(Socket& socket)
        {
            using namespace Navtech::Unit;

            socket.tx_buffer(0_kB);
            socket.tx_timeout(2_sec);

            // For a more detailed explanation behind these settings, visit:
            // https://blog.cloudflare.com/when-tcp-sockets-refuse-to-die/
            //
            socket.keep_alive(true);
            socket.keep_alive_idle(5_sec);
            socket.keep_alive_probes(5);
            socket.keep_alive_interval(1_sec);
            socket.user_timeout(7_sec); 
        }
    };


    // ----------------------------------------------------------------------------------------------------------------
    //
    template <>
    class Connection_traits<Protocol::cat240, Transport::udp, TLS::Type::none> {
    public:
        // Types
        //
        using Socket            = UDP_socket;
        using Message_buffer    = std::vector<std::uint8_t>;

        using Protocol_traits   = Navtech::Networking::Protocol_traits<Protocol::cat240, Transport::udp, TLS::Type::none>;
        using Message           = typename Protocol_traits::Message;
        using ID                = typename Protocol_traits::ID;
   
        // Constants
        //
        static constexpr Unit::Kilobyte max_buffer_size { 4_kB };

        // Functions
        //
        static void set_socket_options(Socket& socket)
        {
            // TODO?
        }
    };


    // ----------------------------------------------------------------------------------------------------------------
    //
    template <>
    class Connection_traits<Protocol::nmea, Transport::udp, TLS::Type::none> {
    public:
        // Types
        //
        using Socket            = UDP_socket;
        using Message_buffer    = std::vector<std::uint8_t>;

        using Protocol_traits   = Navtech::Networking::Protocol_traits<Protocol::nmea, Transport::udp, TLS::Type::none>;
        using Message           = typename Protocol_traits::Message;
        using ID                = typename Protocol_traits::ID;
   
        // Constants
        //
        static constexpr Unit::Kilobyte max_buffer_size { 4_kB };

        // Functions
        //
        static void set_socket_options(Socket& socket)
        {
            // TODO?
        }
    };

} // namespace Navtech::Networking

#endif // CONNECTION_TRAITS_H