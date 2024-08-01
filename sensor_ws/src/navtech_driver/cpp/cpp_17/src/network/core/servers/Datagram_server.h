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
#ifndef DATAGRAM_SERVER_H
#define DATAGRAM_SERVER_H

#include "Datagram_server_traits.h"
#include "Connection_manager.h"
#include "Endpoint.h"
#include "Active.h"
#include "pointer_types.h"
#include "Log.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;

namespace Navtech::Networking {

    template <Protocol protocol, Transport transport = Transport::udp, TLS::Type tls = TLS::Type::none>
    class Datagram_server : public Utility::Active {
    public:
        // Type aliases.
        // The '_Ty' postfix denotes a template type. Rather than being
        // supplied as template parameters on the class (which would be unwieldy) 
        // these parameters are looked up from the Stream_server_traits class, 
        // using the appropriate combination of protocol, transport and TLS type
        //
        using Event_traits      = Navtech::Networking::Event_traits<protocol, transport, tls>;
        using Server_traits     = Datagram_server_traits<protocol, transport, tls>;
        using Connection_Ty     = typename Server_traits::Connection;
        using Socket_Ty         = typename Server_traits::Socket;
        using Socket_ptr_Ty     = typename Server_traits::Socket_ptr;
        using Message_Ty        = typename Server_traits::Message;
        using ID_Ty             = typename Server_traits::ID;
        using Dispatcher_Ty     = typename Event_traits::Dispatcher;

        Datagram_server(Dispatcher_Ty& event_dispatcher);
        Datagram_server(const Endpoint& local_endpt, Dispatcher_Ty& event_dispatcher);
        Datagram_server(const Endpoint& local_endpt, const Endpoint& remote_endpt, Dispatcher_Ty& event_dispatcher);
        Datagram_server(const Endpoint& local_endpt, const Endpoint& remote_endpt, std::uint8_t TTL, Dispatcher_Ty& event_dispatcher);

        void bind_to(const Endpoint& local_endpt);
        void send_to(const Endpoint& remote_endpt);

        void send(const Message_Ty& msg);
        void send(Message_Ty&& msg);

    protected:

    private:
        // External associations
        //
        association_to<Dispatcher_Ty> dispatcher { };

        // NOTE - declaration order is important!
        //        connection is dependent on an initialised socket
        //
        ID_Ty         id;
        Socket_Ty     socket;
        Connection_Ty connection;
        
        // Active object overrides
        //
        void on_start() override;
        void on_stop()  override;

        Error::Dispatcher             error_events  { };
        Utility::Event_handler<ID_Ty> error_handler { };
        void on_error(ID_Ty connection_id);

        // Internal state
        //
        bool enabled { false };

        // Implementation
        //
    };



    template <Protocol protocol, Transport transport, TLS::Type tls>
    Datagram_server<protocol, transport, tls>::Datagram_server(Dispatcher_Ty& event_dispatcher) :
        Active          { "Datagram Server" },
        dispatcher      { associate_with(event_dispatcher) },
        id              { 1 },
        socket          { },
        connection      { id, std::move(socket), event_dispatcher, error_events, Connection_Ty::Direction::tx }
    {
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    Datagram_server<protocol, transport, tls>::Datagram_server(
        const Endpoint& local_endpt, 
        Dispatcher_Ty&  event_dispatcher
    ) :
        Active          { "Datagram Server" },
        dispatcher      { associate_with(event_dispatcher) },
        id              { 1 },
        socket          { local_endpt },
        connection      { id, std::move(socket), event_dispatcher, error_events, Connection_Ty::Direction::tx }

    {
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    Datagram_server<protocol, transport, tls>::Datagram_server(
        const Endpoint& local_endpt, 
        const Endpoint& remote_endpt,
        Dispatcher_Ty&  event_dispatcher
    ) :
        Active          { "Datagram Server" },
        dispatcher      { associate_with(event_dispatcher) },
        id              { 1 },
        socket          { local_endpt, remote_endpt },
        connection      { id, std::move(socket), event_dispatcher, error_events, Connection_Ty::Direction::tx }
    {
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    Datagram_server<protocol, transport, tls>::Datagram_server(
        const Endpoint& local_endpt, 
        const Endpoint& remote_endpt,
        std::uint8_t    TTL,
        Dispatcher_Ty&  event_dispatcher
    ) :
        Active          { "Datagram Server" },
        dispatcher      { associate_with(event_dispatcher) },
        id              { 1 },
        socket          { local_endpt, remote_endpt, TTL },
        connection      { id, std::move(socket), event_dispatcher, error_events, Connection_Ty::Direction::tx }
    {
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Datagram_server<protocol, transport, tls>::bind_to(const Endpoint& local_endpt)
    {
        try {
            socket.bind_to(local_endpt);
        }
        catch (std::system_error& e) {
            stdout_log << Logging_level::debug;
            stdout_log << "Datagram server [" << id << "] failed to bind socket: " << e.what() << endl;
        }
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Datagram_server<protocol, transport, tls>::send_to(const Endpoint& remote_endpt)
    {
        connection.remote_endpoint(remote_endpt);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Datagram_server<protocol, transport, tls>::on_start()
    {
        error_handler.when_notified_invoke(
            [this](ID_Ty connection_id) 
            { 
                async_call(&Datagram_server::on_error, this, connection_id); 
            }
        );

        error_events.attach_to<Error::Event::connection_error>(error_handler);

        enabled = true;
        connection.open();
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Datagram_server<protocol, transport, tls>::on_stop()
    {
        enabled = false;
        error_events.detach_from<Error::Event::connection_error>(error_handler);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Datagram_server<protocol, transport, tls>::on_error(ID_Ty connection_id)
    {
        if (connection_id != id) return; // Error is not for us.

        enabled = false;
        connection.close();
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Datagram_server<protocol, transport, tls>::send(const Message_Ty& msg)
    {
        connection.send(msg);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Datagram_server<protocol, transport, tls>::send(Message_Ty&& msg)
    {
        connection.send(std::move(msg));
    }

} // namespace Navtech::Networking


#endif // DATAGRAM_SERVER_H