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
#ifndef CONNECTION_MANAGER_TRAITS_H
#define CONNECTION_MANAGER_TRAITS_H

#include <vector>
#include <cstdint>

#include "Connection_traits.h"
#include "Messaging_connection.h"
#include "Datagram_connection.h"

#ifdef __linux__
#include "Secure_connection.h"
#endif


namespace Navtech::Networking {

    template <Protocol, Transport, TLS::Type>
    class Connection_manager_traits { };



    // -----------------------------------------------------------------------------------------------------------------
    //
    template <>
    class Connection_manager_traits<Protocol::colossus, Transport::tcp, TLS::Type::none> {
    public:
        // Types
        //
        using Connection        = Navtech::Networking::Messaging::Connection<Protocol::colossus, Transport::tcp, TLS::Type::none>;
        
        using Connection_traits = Navtech::Networking::Connection_traits<Protocol::colossus, Transport::tcp, TLS::Type::none>;
        using Socket            = typename Connection_traits::Socket;
        using Socket_ptr        = shared_owner<Socket>;
        using Connection_ptr    = owner_of<Connection>;
        using ID                = typename Connection_traits::ID;
        using Protocol_traits   = typename Connection_traits::Protocol_traits;

        // Constants
        //
        static constexpr std::size_t max_clients  { 3 };

        // Functions
        //
        static std::set<IP_address> well_known_clients()
        {
            return { };
        }
    };


    // -----------------------------------------------------------------------------------------------------------------
    //
    template <>
    class Connection_manager_traits<Protocol::cp, Transport::tcp, TLS::Type::none> {
    public:
        // Types
        //
        using Connection        = Navtech::Networking::Messaging::Connection<Protocol::cp, Transport::tcp, TLS::Type::none>;
        
        using Connection_traits = Navtech::Networking::Connection_traits<Protocol::cp, Transport::tcp, TLS::Type::none>;
        using Socket            = typename Connection_traits::Socket;
        using Socket_ptr        = shared_owner<Socket>;
        using Connection_ptr    = owner_of<Connection>;
        using ID                = typename Connection_traits::ID;
        using Protocol_traits   = typename Connection_traits::Protocol_traits;

        // Constants
        //
        static constexpr std::size_t max_clients  { 3 };

        // Functions
        //
        static std::set<IP_address> well_known_clients()
        {
            return { };
        }
    };


#ifdef __linux__
    // -----------------------------------------------------------------------------------------------------------------
    //
    template <>
    class Connection_manager_traits<Protocol::cp, Transport::tcp, TLS::Type::botan> {
    public:
        // Types
        //
        using Connection        = Navtech::Networking::Secure::Connection<Protocol::cp, Transport::tcp, TLS::Type::botan>;
        
        using Connection_traits = Navtech::Networking::Connection_traits<Protocol::cp, Transport::tcp, TLS::Type::botan>;
        using Socket            = typename Connection_traits::Socket;
        using Socket_ptr        = shared_owner<Socket>;
        using Connection_ptr    = owner_of<Connection>;
        using ID                = typename Connection_traits::ID;
        using Protocol_traits   = typename Connection_traits::Protocol_traits;

        // Constants
        //
        static constexpr std::size_t max_clients  { 1 };

        // Functions
        //
        static std::set<IP_address> well_known_clients()
        {
            return { };
        }

    };
#endif // __linux__


} // namespace Navtech::Networking

#endif // CONNECTION_MANAGER_TRAITS_H