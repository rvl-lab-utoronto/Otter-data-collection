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
#ifndef CONNECTION_MANAGER_H
#define CONNECTION_MANAGER_H

#include <unordered_map>
#include <set>
#include <memory>

#include "Connection_manager_traits.h"
#include "Event_traits.h"
#include "Connection_error_events.h"
#include "Connection_set.h"

#include "pointer_types.h"
#include "Active.h"
#include "Log.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;


namespace Navtech::Networking {

    // ----------------------------------------------------------------------------
    // The Connection_manager controls the lifetime of Connection_Ty objects, and provides
    // an interface to give access to the current set of Connections.
    //
    template <Protocol protocol, Transport transport, TLS::Type tls>
    class Connection_manager : public Utility::Active {
    public:
        // Type aliases.
        // The '_Ty' postfix denotes a template type. Rather than being
        // supplied as template parameters on the class (which would be unwieldy) 
        // these parameters are looked up from the Stream_server_traits class, 
        // using the appropriate combination of protocol, transport and TLS type
        //
        using Event_traits          = Navtech::Networking::Event_traits<protocol, transport, tls>;
        using Connection_mgr_traits = Connection_manager_traits<protocol, transport, tls>;
        using Protocol_traits       = typename Connection_mgr_traits::Protocol_traits;
        using Connection_Ty         = typename Connection_mgr_traits::Connection;
        using Connection_ptr_Ty     = typename Connection_mgr_traits::Connection_ptr;
        using Socket_Ty             = typename Connection_mgr_traits::Socket;
        using Socket_ptr_Ty         = typename Connection_mgr_traits::Socket_ptr;
        using Message_Ty            = typename Connection_Ty::Message_Ty;
        using ID_Ty                 = typename Connection_mgr_traits::ID;
        using Dispatcher_Ty         = typename Event_traits::Dispatcher;

        Connection_manager(Dispatcher_Ty& protocol_event_dispatcher);

        void create_connection(const Socket_ptr_Ty& socket);
        Connection_set<protocol, transport, tls>&       all_connections();
        const Connection_set<protocol, transport, tls>& all_connections() const;

    protected:
        void on_start() override;
        void on_stop()  override;
        
    private:
        // External associations
        //
        association_to<Dispatcher_Ty>       protocol_events;

        std::set<Networking::IP_address>    well_known_clients { };
        Connection_set<protocol, transport, tls> connections { };

        // Event handling
        //
        Error::Dispatcher               error_dispatcher { };
        Utility::Event_handler<ID_Ty>   error_handler    { };
    
        // Async function implementations
        //
        void on_incoming_connection(const Socket_ptr_Ty& socket);
        void on_error(const ID_Ty& connection_id);

        // Helpers
        //
        ID_Ty next_id();
        bool is_well_known(const IP_address& ip_addr) const;
        bool exceeded_max_connections() const;
        void add_client_connection(Socket_Ty&& sckt);
        void remove_client_connection(ID_Ty id);
        void add_well_known_connection(Socket_Ty&& socket);
        void remove_well_known_connection(ID_Ty id);
        

        static ID_Ty id;
        std::size_t  client_count { };
    };

    // Connection_Ty ID 0 (zero) is configured as the 'broadcast'
    // ID - that is, send to all available connections
    //
    template <Protocol protocol, Transport transport, TLS::Type tls>
    typename Connection_manager<protocol, transport, tls>::ID_Ty Connection_manager<protocol, transport, tls>::id { 1 };

    // ----------------------------------------------------------------------------

    template <Protocol protocol, Transport transport, TLS::Type tls>
    Connection_manager<protocol, transport, tls>::Connection_manager(
        Connection_manager<protocol, transport, tls>::Dispatcher_Ty& protocol_event_dispatcher
    ) :
        Active              { "Connection manager" },
        protocol_events     { associate_with(protocol_event_dispatcher) },
        well_known_clients  { Connection_mgr_traits::well_known_clients() }
    {
    }
    

    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection_manager<protocol, transport, tls>::on_start()
    {
        error_handler.when_notified_invoke(
            [this](ID_Ty connection_id) 
            {
                async_call(&Connection_manager::on_error, this, connection_id); 
            }
        );
        error_dispatcher.attach_to<Error::Event::connection_error>(error_handler);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection_manager<protocol, transport, tls>::on_stop()
    {
        error_dispatcher.detach_from<Error::Event::connection_error>(error_handler);

        connections.remove_all();

        stdout_log << Protocol_traits::name << " connection manager - stopped." << endl;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    Connection_set<protocol, transport, tls>& Connection_manager<protocol, transport, tls>::all_connections()
    {
        return connections;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    const Connection_set<protocol, transport, tls>& Connection_manager<protocol, transport, tls>::all_connections() const
    {
        return connections;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection_manager<protocol, transport, tls>::on_error(const ID_Ty& id)
    {
        IP_address outgoing_client { connections.address_of(id) };
    
        if (is_well_known(outgoing_client)) remove_well_known_connection(id);
        else                                remove_client_connection(id);
    }

   

    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection_manager<protocol, transport, tls>::create_connection(const Socket_ptr_Ty& socket_ptr)
    {
        async_call(&Connection_manager<protocol, transport, tls>::on_incoming_connection, this, socket_ptr);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection_manager<protocol, transport, tls>::on_incoming_connection(const Socket_ptr_Ty& socket_ptr)
    {
        Socket_Ty  socket          { std::move(*socket_ptr) };
        IP_address incoming_client { socket.peer().ip_address };
       
        if (is_well_known(incoming_client)) {
            add_well_known_connection(std::move(socket));
            return;
        }

        if (!exceeded_max_connections()) {
            add_client_connection(std::move(socket));
            return;
        }
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    bool Connection_manager<protocol, transport, tls>::is_well_known(const IP_address& ip_addr) const
    {
        bool well_known { well_known_clients.count(ip_addr) == 1 };

        if (well_known) {
            stdout_log << Protocol_traits::name << " connection manager - "
                       << "connection is well-known address "
                       << "[" << ip_addr.to_string() << "]"
                       << endl;
        }

        return well_known;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    bool Connection_manager<protocol, transport, tls>::exceeded_max_connections() const
    {
        bool client_count_exceeded { client_count >= Connection_mgr_traits::max_clients };

        if (client_count_exceeded) {
            stdout_log << Protocol_traits::name << " connection manager - "
                       << "maximum connections exceeded "
                       << "[" << Connection_mgr_traits::max_clients << "]"
                       << endl;
        }

        return client_count_exceeded;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection_manager<protocol, transport, tls>::add_client_connection(Connection_manager<protocol, transport, tls>::Socket_Ty&& socket)
    {
        auto id = next_id();
        ++client_count;

        stdout_log << Protocol_traits::name << " connection manager - "
                   << "created new connection "
                   << "[" << id << "]"
                   << "[" << socket.peer().ip_address.to_string() << "] "
                   << "[" << client_count << " of " << Connection_mgr_traits::max_clients << "]"
                   << endl;

        auto connection = allocate_owned<Connection_Ty>(
            id, 
            std::move(socket),
            *protocol_events,
            error_dispatcher
        );

        connection->open();
        connections.add(std::move(connection));
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection_manager<protocol, transport, tls>::remove_client_connection(ID_Ty id)
    {
        bool removed { connections.remove(id) };

        // The connection has probably already been removed
        //
        if (!removed) return;

        --client_count;

        stdout_log << Protocol_traits::name << " connection manager - "
                   << "removed connection [" << id << "] "
                   << "Current connection count [" << client_count << "]"
                   << endl;

        protocol_events->template notify<Event_traits::Client_disconnected>(id);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection_manager<protocol, transport, tls>::add_well_known_connection(Connection_manager<protocol, transport, tls>::Socket_Ty&& socket)
    {
        auto incoming_client  = socket.peer().ip_address;
        auto [exists, old_id] = connections.exists(incoming_client);

        auto new_id = next_id();

        auto connection = allocate_owned<Connection_Ty>( 
            new_id, 
            std::move(socket),
            *protocol_events,
            error_dispatcher
        );

        connection->open();

        if (exists) {
            stdout_log << Protocol_traits::name << " connection manager - "
                       << "replaced connection "
                       << "[" << old_id << "] "
                       << "with "
                       << "[" << new_id << "]" 
                       << "[" << incoming_client.to_string() << "]"
                       << endl;

            connections.replace(old_id, std::move(connection));
        }
        else {
            stdout_log << Protocol_traits::name << " connection manager - "
                       << "created new connection "
                       << "[" << new_id << "]"
                       << "[" << socket.peer().ip_address.to_string() + "]"
                       << endl;

            connections.add(std::move(connection));
        }

        protocol_events->template notify<Event_traits::Client_connected>(new_id);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection_manager<protocol, transport, tls>::remove_well_known_connection(ID_Ty id)
    {
        // NOTE: 
        // A well-known connection does not change the 
        // number of client connections

        bool removed { connections.remove(id) };

        // The connection has probably already been removed
        //
        if (!removed) return;

        stdout_log << Protocol_traits::name << " connection manager - removed connection [" << id << "]" << endl;

        protocol_events->template notify<Event_traits::Client_disconnected>(id);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    typename Connection_manager<protocol, transport, tls>::ID_Ty Connection_manager<protocol, transport, tls>::next_id()
    {
        return id++;
    }


} // namespace Navtech::Networking

#endif // CONNECTION_MANAGER_H