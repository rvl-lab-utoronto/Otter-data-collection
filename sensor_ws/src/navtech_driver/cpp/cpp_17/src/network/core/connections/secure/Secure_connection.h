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
#ifndef SECURE_CONNECTION_H
#define SECURE_CONNECTION_H

#include "Connection_traits.h"

#include "Secure_marshaller.h"
#include "Secure_parser.h"
#include "Secure_sender.h"
#include "Secure_receiver.h"
#include "Botan_TLS_services.h"
#include "Botan_TLS_server.h"

#include "Active.h"
#include "pointer_types.h"
#include "string_helpers.h"

using namespace Navtech::Time;

namespace Navtech::Networking::Secure {

    template <Protocol protocol, Transport transport = Transport::tcp, TLS::Type tls = TLS::Type::none>
    class Connection {
    public:
        // Type aliases.
        // The '_Ty' postfix denotes a template type. Rather than being
        // supplied as template parameters on the class (which would be unwieldy) 
        // these parameters are looked up from the Stream_server_traits class, 
        // using the appropriate combination of protocol, transport and TLS type
        //
        using Event_traits      = Navtech::Networking::Event_traits<protocol, transport, tls>;
        using Connection_traits = Navtech::Networking::Connection_traits<protocol, transport, tls>;
        using Protocol_traits   = typename Connection_traits::Protocol_traits;
        using Socket_Ty         = typename Connection_traits::Socket;
        using Message_Ty        = typename Connection_traits::Message;
        using ID_Ty             = typename Connection_traits::ID;
        using Dispatcher_Ty     = typename Event_traits::Dispatcher;

        Connection(
            ID_Ty               identifier, 
            Socket_Ty&&         sckt, 
            Dispatcher_Ty&      protocol_event_dispatcher,
            Error::Dispatcher&  error_event_dispatcher
        );
        ~Connection();

        void open();
        void close();

        void bind_to(const Endpoint& local_endpt);
        void remote_endpoint(const Endpoint& remote_endpt);
        Endpoint remote_endpoint() const;

        void send(const Message_Ty& msg);
        void send(Message_Ty&& msg);

        bool is_enabled() const;

        ID_Ty id() const;

    private:
        // External associations
        //
        association_to<Dispatcher_Ty>       protocol_events;
        association_to<Error::Dispatcher>   error_events;

        // Internal components
        //
        Socket_Ty socket { };
        
        Marshaller<protocol, transport, tls> marshaller;
        Parser<protocol, transport, tls>     parser;
        TLS::Botan::Server                   tls_server;                 

        Sender<protocol, transport, tls>     sender;
        Receiver<protocol, transport, tls>   receiver;

        // Event handling (from sender and receiver)
        //
        Utility::Event_handler<ID_Ty> tx_error_handler { };
        Utility::Event_handler<ID_Ty> rx_error_handler { };
        void on_send_error(const ID_Ty& identity);
        void on_receive_error(const ID_Ty& identity);
        void on_tls_error(const ID_Ty& identity);

        // Internal state
        //
        ID_Ty             ident         { };
        Endpoint          remote_endpt  { };
        std::atomic<bool> enabled       { false };
    };


    template <Protocol protocol, Transport transport, TLS::Type tls>
    Connection<protocol, transport, tls>::Connection(
        Connection<protocol, transport, tls>::ID_Ty            identity, 
        Connection<protocol, transport, tls>::Socket_Ty&&      sckt,
        Connection<protocol, transport, tls>::Dispatcher_Ty&   protocol_event_dispatcher,
        Error::Dispatcher&                                     error_event_dispatcher
    ) :
        protocol_events { associate_with(protocol_event_dispatcher) },
        error_events    { associate_with(error_event_dispatcher) },
        socket          { std::move(sckt) },
        
        marshaller      { },
        parser          { identity, protocol_event_dispatcher },
        tls_server      { TLS::Botan::TLS_services::make() },

        sender          { socket, identity, protocol_event_dispatcher, error_event_dispatcher },
        receiver        { socket, identity, protocol_event_dispatcher, error_event_dispatcher },

        ident           { identity },
        remote_endpt    { socket.peer() }
    {
        try {
            Connection_traits::set_socket_options(socket);
        }
        catch (std::system_error& e) {
            stdout_log << "Secure connection [" << ident << "] "
                       << "error setting socket options: " 
                       << e.what()
                       << endl;
        }

        // Wire everything up...
        //
        marshaller.out.forward_to(tls_server.encrypt_in);
        tls_server.encrypt_out.forward_to(sender.in);

        receiver.out.forward_to(tls_server.decrypt_in);
        tls_server.decrypt_out.forward_to(parser.in);

        // TLS error handling
        //
        tls_server.error.on_receive(
            [this](const int& id) 
            {
                on_tls_error(id); 
            }
        );

        // TLS connection status handling
        //
        tls_server.status.on_receive(
            [this](const TLS::Botan::Server::Status& status)
            {
                // Once the TLS server reports 'activated' the connection
                // is ready for use. Inform any higher-level clients.
                // 
                if (status == TLS::Botan::Server::activated) {
                    enabled = true;
                    protocol_events->template notify<Event_traits::Client_connected>(ident);
                }
                else {
                    enabled = false;
                }
            }
        );

    }

    
    template <Protocol protocol, Transport transport, TLS::Type tls>
    Connection<protocol, transport, tls>::~Connection()
    {
        close();
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection<protocol, transport, tls>::open()
    {
        tx_error_handler.when_notified_invoke(&Connection::on_send_error, this);
        rx_error_handler.when_notified_invoke(&Connection::on_receive_error, this);
        error_events->attach_to<Error::Event::tx_error>(tx_error_handler);
        error_events->attach_to<Error::Event::rx_error>(rx_error_handler);

        // The TLS server requires a network connection
        // as soon as it is started. Therefore, the Sender
        // and Receiver must be started before attempting
        // to start the TLS server
        //
        marshaller.start();
        parser.start();
        sender.start();
        receiver.start();
        tls_server.start();

        enabled = true;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection<protocol, transport, tls>::close()
    {
        if (!enabled) return;
        enabled = false;

        error_events->detach_from<Error::Event::tx_error>(tx_error_handler);
        error_events->detach_from<Error::Event::rx_error>(rx_error_handler);

        // Closing the socket will cause any currently-block reads or writes
        // to fail; allowing the sender/receiver to exit
        //
        try {
            socket.close();
        } 
        catch (...) {
           // We're closing, so ignore any errors
           // the socket may throw.
        }

        marshaller.stop();
        parser.stop();

        sender.stop();
        receiver.stop();
        receiver.join();

        tls_server.stop();
        tls_server.join();

        // Diagnostics
        //
        stdout_log << "Secure connection [" << ident << "] closed. "
                   << "Bytes received [" << Utility::to_memory_string(socket.bytes_read()) << "] "
                   << "Bytes sent ["     << Utility::to_memory_string(socket.bytes_sent()) << "]"
                   << endl;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection<protocol, transport, tls>::send(const Message_Ty& msg)
    {
        if (!enabled) return;

        marshaller.send(msg);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection<protocol, transport, tls>::remote_endpoint(const Endpoint& remote_endpt)
    {
        socket.remote_endpoint(remote_endpt);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    Endpoint Connection<protocol, transport, tls>::remote_endpoint() const
    {
        return remote_endpt;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection<protocol, transport, tls>::send(Message_Ty&& msg)
    {
        if (!enabled) return;

        marshaller.send(std::move(msg)); 
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection<protocol, transport, tls>::on_send_error(const ID_Ty& session_in_error)
    {
        if (!enabled)                        return;
        if (session_in_error != this->ident) return;

        // The Sender will have already disabled itself
        // so disable the Receiver explicitly
        //
        receiver.disable();

        // The receiver of the connection_error event 
        // should terminate this connection.
        //
        stdout_log << Logging_level::debug;
        stdout_log << "Secure connection [" << ident << "] send error.  Requesting termination" << endl;
        error_events->notify<Error::Event::connection_error>(ident);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection<protocol, transport, tls>::on_receive_error(const ID_Ty& session_in_error)
    {
        if (!enabled)                        return;
        if (session_in_error != this->ident) return;

        // The Receiver will have already disabled itself
        // so disable the Sender explicitly
        //
        sender.disable();
    
        // The receiver of the connection_error event 
        // should terminate this connection.
        //
        stdout_log << Logging_level::debug;
        stdout_log << "Secure connection [" << ident << "] receive error.  Requesting termination" << endl;
        error_events->notify<Error::Event::connection_error>(ident);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection<protocol, transport, tls>::on_tls_error(const ID_Ty& session_in_error)
    {
        if (!enabled)                        return;
        if (session_in_error != this->ident) return;

        // The sender and receiver may still be active
        //
        sender.disable();
        receiver.disable();

        // The receiver of the connection_error event 
        // should terminate this connection.
        //
        stdout_log << Logging_level::debug;
        stdout_log << "Secure connection [" << ident << "] TLS error.  Requesting termination" << endl;
        error_events->notify<Error::Event::connection_error>(ident);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    typename Connection<protocol, transport, tls>::ID_Ty 
    Connection<protocol, transport, tls>::id() const
    {
        return ident;
    }

} // namespace Navtech::Networking::Secure

#endif // SECURE_CONNECTION_H