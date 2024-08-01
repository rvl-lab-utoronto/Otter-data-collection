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
#ifndef STREAM_SERVER_H
#define STREAM_SERVER_H

#include "Stream_server_traits.h"
#include "Event_traits.h"
#include "Acceptor.h"
#include "Connection_manager.h"
#include "Endpoint.h"
#include "Active.h"
#include "Log.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;


namespace Navtech::Networking {


    template <Protocol protocol, Transport transport = Transport::tcp, TLS::Type tls = TLS::Type::none>
    class Stream_server {
    public:
        // Type aliases.
        // The '_Ty' postfix denotes a template type. Rather than being
        // supplied as template parameters on the class (which would be
        // unwieldy) these parameters are looked up from the Stream_server_traits
        // class, using the appropriate combination of protocol and transport
        //
        using Event_traits      = Navtech::Networking::Event_traits<protocol, transport, tls>;
        using Server_traits     = Stream_server_traits<protocol, transport, tls>;
        using Protocol_traits   = typename Server_traits::Protocol_traits;
        using Socket_Ty         = typename Server_traits::Socket;
        using Socket_ptr_Ty     = typename Server_traits::Socket_ptr;
        using Message_Ty        = typename Server_traits::Message;
        using ID_Ty             = typename Server_traits::ID;
        using Dispatcher_Ty     = typename Event_traits::Dispatcher;

        Stream_server(Dispatcher_Ty& protocol_event_dispatcher);
        Stream_server(Port port, Dispatcher_Ty& protocol_event_dispatcher);

        void bind_to(Port port);

        void start();
        void stop();

        void send(const Message_Ty& msg);
        void send(Message_Ty&& msg);

        std::size_t client_count() const;

    protected:
        friend class Acceptor<protocol, transport, tls>;

        // Asynchronous interface
        //
        void start_connection(const Socket_ptr_Ty& socket);
        void start_connection_impl(const Socket_ptr_Ty& socket);

    private:
        // External associations
        //
        association_to<Dispatcher_Ty>   protocol_events;

        // Error event handling
        //
        
        // Internal components
        // NOTE - Do not change the order of these declarations!
        //        there is a dependency between them!
        Connection_manager<protocol, transport, tls> connection_mgr;
        Acceptor<protocol, transport, tls>           acceptor;
       
        // Internal state
        //
        bool enabled { false };

        // Send implementation
        //
        Utility::Event_handler<Message_Ty> send_handler { };    
    };


    // ----------------------------------------------------------------------------------------------------------------
    //
    template <Protocol protocol, Transport transport, TLS::Type tls>
    Stream_server<protocol, transport, tls>::Stream_server(
        Stream_server<protocol, transport, tls>::Dispatcher_Ty& protocol_event_dispatcher
    ) :
        Active          { "Stream Server" },
        protocol_events { associate_with(protocol_event_dispatcher) },
        connection_mgr  { protocol_event_dispatcher },
        acceptor        { connection_mgr } 
    {
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    Stream_server<protocol, transport, tls>::Stream_server(
        Port                                                    port,
        Stream_server<protocol, transport, tls>::Dispatcher_Ty& protocol_event_dispatcher
    ) :
        protocol_events { associate_with(protocol_event_dispatcher) },
        connection_mgr  { protocol_event_dispatcher },
        acceptor        { connection_mgr, port }
    {
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Stream_server<protocol, transport, tls>::bind_to(Port port)
    {
        acceptor.bind_to(port);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Stream_server<protocol, transport, tls>::start()
    {
        stdout_log << Server_traits::Protocol_traits::name << " stream server - starting..." << endl;

        send_handler.when_notified_invoke(&Stream_server<protocol, transport, tls>::send, this);
        protocol_events->template attach_to<Event_traits::Send_message>(send_handler);
        
        connection_mgr.start();
        acceptor.start();

        enabled = true;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Stream_server<protocol, transport, tls>::stop()
    {
        stdout_log << Server_traits::Protocol_traits::name << " stream server - stopping..." << endl;

        enabled = false;

        protocol_events->template detach_from<Event_traits::Send_message>(send_handler);

        acceptor.stop();
        acceptor.join();

        connection_mgr.stop();
        connection_mgr.join();

        stdout_log << Server_traits::Protocol_traits::name << " stream server - stopped." << endl;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Stream_server<protocol, transport, tls>::send(const Message_Ty& msg)
    {
        if (!enabled) return;

        auto& connections = connection_mgr.all_connections();
        connections.send(msg);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Stream_server<protocol, transport, tls>::send(Message_Ty&& msg)
    {
       if (!enabled) return;

        auto& connections = connection_mgr.all_connections();
        connections.send(std::move(msg));
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    std::size_t Stream_server<protocol, transport, tls>::client_count() const
    {
       return connection_mgr.all_connections().size();
    }

} // namespace Navtech::Networking

#endif // STREAM_SERVER_H