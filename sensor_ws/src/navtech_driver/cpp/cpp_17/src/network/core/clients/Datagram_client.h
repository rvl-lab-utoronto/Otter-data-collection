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
#ifndef DATAGRAM_CLIENT_H
#define DATAGRAM_CLIENT_H

#include "Datagram_client_traits.h"
#include "Event_traits.h"
#include "Active.h"
#include "Endpoint.h"
#include "Time_utils.h"
#include "Log.h"
#include "Connection_manager.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;


namespace Navtech::Networking {

    // The Datagram_client maintains the lifetime of a server datagram connection
    //
    template <Protocol protocol, Transport transport, TLS::Type tls = TLS::Type::none>
    class Datagram_client : public Utility::Active {
    public:
        // Type aliases.
        // The '_Ty' postfix denotes a template type. Rather than being
        // supplied as template parameters on the class (which would be
        // unwieldy) these parameters are looked up from the Networking::Traits
        // class, using the appropriate combination of protocol and transport
        //
        using Traits            = Navtech::Networking::Datagram_client_traits<protocol, transport, tls>;
        using Event_traits      = Navtech::Networking::Event_traits<protocol, transport, tls>;
        using Socket_Ty         = typename Traits::Socket;
        using ID_Ty             = typename Traits::ID;
        using Message_Ty        = typename Traits::Message;
        using Dispatcher_Ty     = typename Event_traits::Dispatcher;
        
        Datagram_client(const Endpoint& recv_from, Dispatcher_Ty& event_dispatcher);

    private:
        // External associations
        //
        association_to<Dispatcher_Ty> dispatcher;

        // NOTE - declaration order is important!
        //        connection is dependent on an initialised socket
        //
        ID_Ty         id;
        Socket_Ty  socket;

        using Connection = Datagram::Connection<protocol, transport, tls>;

        Connection connection;

        // Active class overrides
        //
        void on_start() override;
        void on_stop()  override;

        // Event handling
        //
        Error::Dispatcher             error_events  { };
        Utility::Event_handler<ID_Ty> error_handler { };
        void on_error(ID_Ty connection_id);

        // Internal state
        //
        bool enabled { false };
    };


    // -----------------------------------------------------------------------------------------------------------------
    //
    template <Protocol protocol, Transport transport, TLS::Type tls>
    Datagram_client<protocol, transport, tls>::Datagram_client(
        const Endpoint&                                      recv_from, 
        Datagram_client<protocol, transport, tls>::Dispatcher_Ty& event_dispatcher
    ) :
        Active          { "Datagram client" },
        dispatcher      { associate_with(event_dispatcher) },
        id              { 1 },
        socket          { Endpoint { recv_from.port }, recv_from },
        connection      { id, std::move(socket), event_dispatcher, error_events, Connection::Direction::rx }
    {
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Datagram_client<protocol, transport, tls>::on_start()
    {
        error_handler.when_notified_invoke(
            [this](ID_Ty connection_id) 
            { 
                async_call(&Datagram_client::on_error, this, connection_id); 
            }
        );
        
        error_events.attach_to<Error::Event::connection_error>(error_handler);

        enabled = true;
        connection.open();
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Datagram_client<protocol, transport, tls>::on_stop()
    {
        enabled = false;
        connection.close();
        error_events.detach_from<Error::Event::connection_error>(error_handler);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Datagram_client<protocol, transport, tls>::on_error(ID_Ty connection_id)
    {
        if (connection_id != id) return; // Error is not for us.

        enabled = false;
        connection.close();
    }

} // namespace Navtech::Networking

#endif // DATAGRAM_CLIENT_H