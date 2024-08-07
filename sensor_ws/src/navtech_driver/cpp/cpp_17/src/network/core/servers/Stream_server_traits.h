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
#ifndef STREAM_SERVER_TRAITS
#define STREAM_SERVER_TRAITS

#include "Connection_traits.h"
#include "TCP_socket.h"

#include "Time_utils.h"


namespace Navtech::Networking {

    template <Protocol, Transport, TLS::Type>
    class Stream_server_traits { };


    // ----------------------------------------------------------------------------------------------------------------
    //
    template <>
    class Stream_server_traits<Protocol::colossus, Transport::tcp, TLS::Type::none> {
    public:
        // Types
        //
        using Connection_traits = Navtech::Networking::Connection_traits<Protocol::colossus, Transport::tcp, TLS::Type::none>;
        using Socket            = typename Connection_traits::Socket;
        using Socket_ptr        = shared_owner<Socket>;
        using Message           = typename Connection_traits::Message;
        using ID                = typename Connection_traits::ID;
        using Protocol_traits   = typename Connection_traits::Protocol_traits;

        // Constants
        //
        static constexpr std::size_t max_connections    { 3 };
        static constexpr std::size_t listener_backlog   { 128 };

        // Functions
        //
        template <typename... Arg_Ty>
        static Socket_ptr allocate_socket(Arg_Ty&&... arg)
        {
            return allocate_shared<Socket>(std::forward<Arg_Ty>(arg)...);
        }
    };


    // ----------------------------------------------------------------------------------------------------------------
    //
    template <>
    class Stream_server_traits<Protocol::cp, Transport::tcp, TLS::Type::none> {
    public:
        // Types
        //
        using Connection_traits = Navtech::Networking::Connection_traits<Protocol::cp, Transport::tcp, TLS::Type::none>;
        using Socket            = typename Connection_traits::Socket;
        using Socket_ptr        = shared_owner<Socket>;
        using Message           = typename Connection_traits::Message;
        using ID                = typename Connection_traits::ID;
        using Protocol_traits   = typename Connection_traits::Protocol_traits;

        // Constants
        //
        static constexpr std::size_t max_connections    { 3 };
        static constexpr std::size_t listener_backlog   { 128 };

        // Functions
        //
        template <typename... Arg_Ty>
        static Socket_ptr allocate_socket(Arg_Ty&&... arg)
        {
            return allocate_shared<Socket>(std::forward<Arg_Ty>(arg)...);
        }
    };


    // ----------------------------------------------------------------------------------------------------------------
    //
    template <>
    class Stream_server_traits<Protocol::cp, Transport::tcp, TLS::Type::botan> {
    public:
        // Types
        //
        using Connection_traits = Navtech::Networking::Connection_traits<Protocol::cp, Transport::tcp, TLS::Type::botan>;
        using Socket            = typename Connection_traits::Socket;
        using Socket_ptr        = shared_owner<Socket>;
        using Message           = typename Connection_traits::Message;
        using ID                = typename Connection_traits::ID;
        using Protocol_traits   = typename Connection_traits::Protocol_traits;

        // Constants
        //
        static constexpr std::size_t max_connections    { 1 };
        static constexpr std::size_t listener_backlog   { 128 };

        // Functions
        //
        template <typename... Arg_Ty>
        static Socket_ptr allocate_socket(Arg_Ty&&... arg)
        {
            return allocate_shared<Socket>(std::forward<Arg_Ty>(arg)...);
        }
    };

} // namespace Navtech::Networking


#endif // STREAM_SERVER_TRAITS