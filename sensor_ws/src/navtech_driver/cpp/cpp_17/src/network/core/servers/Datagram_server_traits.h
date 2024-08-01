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
#ifndef DATAGRAM_SERVER_TRAITS
#define DATAGRAM_SERVER_TRAITS


#include "Connection_traits.h"
#include "Datagram_connection.h"
#include "TCP_socket.h"

#include "Time_utils.h"


namespace Navtech::Networking {

    template <Protocol, Transport, TLS::Type>
    class Datagram_server_traits { };


    // -----------------------------------------------------------------------------------------------------------------
    //
    template <>
    class Datagram_server_traits<Protocol::cat240, Transport::udp, TLS::Type::none> {
    public:
        // Types
        //
        using Connection_traits = Navtech::Networking::Connection_traits<Protocol::cat240, Transport::udp, TLS::Type::none>;
        using Connection        = Navtech::Networking::Datagram::Connection<Protocol::cat240, Transport::udp, TLS::Type::none>;
        using Socket            = typename Connection_traits::Socket;
        using Socket_ptr        = shared_owner<Socket>;
        using Message           = typename Connection_traits::Message;
        using ID                = typename Connection_traits::ID;
        using Protocol_traits   = typename Connection_traits::Protocol_traits;
    
        // Constants
        //

        // Functions
        //
        template <typename... Arg>
        static Socket_ptr allocate_socket(Arg&&... arg)
        {
            return allocate_shared<Socket>(std::forward<Arg>(arg)...);
        }
    };


    // -----------------------------------------------------------------------------------------------------------------
    //
    template <>
    class Datagram_server_traits<Protocol::colossus, Transport::udp, TLS::Type::none> {
    public:
        // Types.
        //
        using Connection_traits = Navtech::Networking::Connection_traits<Protocol::colossus, Transport::udp, TLS::Type::none>;
        using Protocol_traits   = typename Connection_traits::Protocol_traits;
        using Connection        = Navtech::Networking::Datagram::Connection<Protocol::colossus, Transport::udp, TLS::Type::none>;
        using Socket            = typename Connection_traits::Socket;
        using Socket_ptr        = shared_owner<Socket>;
        using Message           = typename Connection_traits::Message;
        using ID                = typename Connection_traits::ID;
        
        // Constants
        //
    
        // Functions
        //
        template <typename... Arg>
        static Socket_ptr allocate_socket(Arg&&... arg)
        {
            return allocate_shared<Socket>(std::forward<Arg>(arg)...);
        }
    };


    // -----------------------------------------------------------------------------------------------------------------
    //
    template <>
    class Datagram_server_traits<Protocol::nmea, Transport::udp, TLS::Type::none> {
    public:
        // Types.
        //
        using Connection_traits = Navtech::Networking::Connection_traits<Protocol::nmea, Transport::udp, TLS::Type::none>;
        using Protocol_traits   = typename Connection_traits::Protocol_traits;
        using Connection        = Navtech::Networking::Datagram::Connection<Protocol::nmea, Transport::udp, TLS::Type::none>;
        using Socket            = typename Connection_traits::Socket;
        using Socket_ptr        = shared_owner<Socket>;
        using Message           = typename Connection_traits::Message;
        using ID                = typename Connection_traits::ID;
        
        // Constants
        //
    
        // Functions
        //
        template <typename... Arg>
        static Socket_ptr allocate_socket(Arg&&... arg)
        {
            return allocate_shared<Socket>(std::forward<Arg>(arg)...);
        }
    };


} // namespace Navtech::Networking


#endif // DATAGRAM_SERVER_TRAITS