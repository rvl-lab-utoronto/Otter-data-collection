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
#ifndef EVENT_TRAITS_H
#define EVENT_TRAITS_H

#include "Colossus_protocol_traits.h"
#include "Colossus_TCP_events.h"
#include "Colossus_UDP_events.h"

#include "CP_protocol_traits.h"
#include "CP_events.h"

#include "Cat240_protocol_traits.h"
#include "Cat240_events.h"

#include "NMEA_protocol_traits.h"
#include "NMEA_events.h"

namespace Navtech::Networking {

    template <Protocol, Transport, TLS::Type>
    class Event_traits { };

    // ------------------------------------------------------------------------------------------------
    //
    template <>
    class Event_traits<Protocol::colossus, Transport::tcp, TLS::Type::none> {
    public:
        using Protocol_traits   = Navtech::Networking::Protocol_traits<Protocol::colossus, Transport::tcp, TLS::Type::none>;
        using Dispatcher        = Colossus_protocol::TCP::Event_dispatcher;
        using Event             = Colossus_protocol::TCP::Event;

        static Dispatcher dispatcher;

        static constexpr Event Send_message               { Colossus_protocol::TCP::Event::send_message };
        static constexpr Event Received_message           { Colossus_protocol::TCP::Event::received_message };
        static constexpr Event Client_connected           { Colossus_protocol::TCP::Event::client_connected };
        static constexpr Event Client_disconnected        { Colossus_protocol::TCP::Event::client_disconnected };
    };


    // ------------------------------------------------------------------------------------------------
    //
    template <>
    class Event_traits<Protocol::colossus, Transport::udp, TLS::Type::none> {
    public:
        using Protocol_traits   = Navtech::Networking::Protocol_traits<Protocol::colossus, Transport::udp, TLS::Type::none>;
        using Dispatcher        = Colossus_protocol::UDP::Event_dispatcher;
        using Event             = Colossus_protocol::UDP::Event;

        static Dispatcher dispatcher;

        static constexpr Event Send_message               { Colossus_protocol::UDP::Event::send_message };
        static constexpr Event Received_message           { Colossus_protocol::UDP::Event::received_message };
        static constexpr Event Client_connected           { Colossus_protocol::UDP::Event::client_connected };
        static constexpr Event Client_disconnected        { Colossus_protocol::UDP::Event::client_disconnected };
    };


    // ------------------------------------------------------------------------------------------------
    //
    template <>
    class Event_traits<Protocol::cp, Transport::tcp, TLS::Type::botan> {
    public:
        using Protocol_traits   = Navtech::Networking::Protocol_traits<Protocol::cp, Transport::tcp, TLS::Type::botan>;
        using Dispatcher        = CP_protocol::Event_dispatcher;
        using Event             = CP_protocol::Event;

        static Dispatcher dispatcher;

        static constexpr Event Send_message               { CP_protocol::Event::send_message };
        static constexpr Event Received_message           { CP_protocol::Event::received_message };
        static constexpr Event Client_connected           { CP_protocol::Event::client_connected };
        static constexpr Event Client_disconnected        { CP_protocol::Event::client_disconnected };
    };


    // ------------------------------------------------------------------------------------------------
    //
    template <>
    class Event_traits<Protocol::cp, Transport::tcp, TLS::Type::none> {
    public:
        using Protocol_traits   = Navtech::Networking::Protocol_traits<Protocol::cp, Transport::tcp, TLS::Type::none>;
        using Dispatcher        = CP_protocol::Event_dispatcher;
        using Event             = CP_protocol::Event;

        static Dispatcher dispatcher;

        static constexpr Event Send_message               { CP_protocol::Event::send_message };
        static constexpr Event Received_message           { CP_protocol::Event::received_message };
        static constexpr Event Client_connected           { CP_protocol::Event::client_connected };
        static constexpr Event Client_disconnected        { CP_protocol::Event::client_disconnected };
    };


    // ------------------------------------------------------------------------------------------------
    //
    template <>
    class Event_traits<Protocol::cat240, Transport::udp, TLS::Type::none> {
    public:
        using Protocol_traits   = Navtech::Networking::Protocol_traits<Protocol::cat240, Transport::udp, TLS::Type::none>;
        using Dispatcher        = Cat240_protocol::Event_dispatcher;
        using Event             = Cat240_protocol::Event;

        static Dispatcher dispatcher;

        static constexpr Event Send_message               { Cat240_protocol::Event::send_message };
        static constexpr Event Received_message           { Cat240_protocol::Event::received_message };
        static constexpr Event Client_connected           { Cat240_protocol::Event::client_connected };
        static constexpr Event Client_disconnected        { Cat240_protocol::Event::client_disconnected };
    };
    

    // ------------------------------------------------------------------------------------------------
    //
    template <>
    class Event_traits<Protocol::nmea, Transport::udp, TLS::Type::none> {
    public:
        using Protocol_traits   = Navtech::Networking::Protocol_traits<Protocol::nmea, Transport::udp, TLS::Type::none>;
        using Dispatcher        = NMEA_protocol::Event_dispatcher;
        using Event             = NMEA_protocol::Event;

        static Dispatcher dispatcher;

        static constexpr Event Send_message               { NMEA_protocol::Event::send_message };
        static constexpr Event Received_message           { NMEA_protocol::Event::received_message };
        static constexpr Event Client_connected           { NMEA_protocol::Event::client_connected };
        static constexpr Event Client_disconnected        { NMEA_protocol::Event::client_disconnected };
    };

} // namespace Navtech::Networking


#endif // EVENT_TRAITS_H