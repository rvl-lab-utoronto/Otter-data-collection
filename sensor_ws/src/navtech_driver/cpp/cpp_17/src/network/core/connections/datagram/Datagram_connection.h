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
#ifndef DATAGRAM_CONNECTION_H
#define DATAGRAM_CONNECTION_H

#include <atomic>

#include "Connection_traits.h"
#include "Event_traits.h"
#include "Connection_error_events.h"
#include "Active.h"
#include "pointer_types.h"
#include "Time_utils.h"
#include "socket_exceptions.h"
#include "string_helpers.h"

#include "Log.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;
using Navtech::Utility::Active;


using namespace Navtech::Time;

namespace Navtech::Networking::Datagram {

    // =================================================================================================================
    //
    template <Protocol protocol, Transport transport = Transport::tcp, TLS::Type tls = TLS::Type::none>
    class Sender {
    public:
        // Type aliases.
        // The '_Ty' postfix denotes a template type. Rather than being
        // supplied as template parameters on the class (which would be
        // unwieldy) these parameters are looked up from the Connection_traits
        // class, using the appropriate combination of protocol and transport
        //
        using Event_traits      = Navtech::Networking::Event_traits<protocol, transport, tls>;
        using Connection_traits = Navtech::Networking::Connection_traits<protocol, transport, tls>;
        using Protocol_traits   = typename Connection_traits::Protocol_traits;
        using Socket_Ty         = typename Connection_traits::Socket;
        using Message_buffer_Ty = typename Connection_traits::Message_buffer;
        using ID_Ty             = typename Connection_traits::ID;
        using Dispatcher_Ty     = typename Event_traits::Dispatcher;

        Sender(
            Socket_Ty&          sckt, 
            ID_Ty               id, 
            Dispatcher_Ty&      event_dispatcher,
            Error::Dispatcher&  error_event_dispatcher
        );

        void start();
        void stop();

        void send(const Message_buffer& msg);
        void send(Message_buffer&& msg);

        void enable();
        void disable();
    
    private:
        // External service associations
        //
        association_to<Socket_Ty>           socket;
        association_to<Dispatcher_Ty>       protocol_events;
        association_to<Error::Dispatcher>   error_events;

        // Operating state
        //
        std::atomic<bool> enabled { false };
        ID_Ty id;

        // Async function implementation
        //
        void do_send(Message_buffer& msg);
    };


template <Protocol protocol, Transport transport, TLS::Type tls>
    Sender<protocol, transport, tls>::Sender(
        Sender<protocol, transport, tls>::Socket_Ty&       sckt, 
        Sender<protocol, transport, tls>::ID_Ty            identity, 
        Sender<protocol, transport, tls>::Dispatcher_Ty&   event_dispatcher,
        Error::Dispatcher&                                 error_event_dispatcher
    ) :
        socket              { associate_with(sckt) },
        protocol_events     { associate_with(event_dispatcher) },
        error_events        { associate_with(error_event_dispatcher) },
        id                  { identity }
    {
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Sender<protocol, transport, tls>::start()
    {
        enable();
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Sender<protocol, transport, tls>::stop()
    {
        disable();
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Sender<protocol, transport, tls>::send(const Message_buffer& msg)
    {
        if (!enabled) return;

        try {
            socket->send(msg);
        }
        catch (client_shutdown&) {
            stdout_log << Logging_level::debug;
            stdout_log << "Datagram sender [" << id << "] client disconnected" << endl;

            disable();
            error_events->notify<Error::Event::tx_error>(id);
        }
        catch (std::system_error& ex) {
            stdout_log << Logging_level::debug;
            stdout_log << "Datagram sender [" << id << "] send() caught exception" << ex.what() << endl;
        
            disable();
            error_events->notify<Error::Event::tx_error>(id);
        }
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Sender<protocol, transport, tls>::send(Message_buffer&& msg)
    {
        if (!enabled) return;

        try {
            socket->send(std::move(msg));
        }
        catch (client_shutdown&) {
            stdout_log << Logging_level::debug;
            stdout_log << "Datagram sender [" << id << "] client disconnected" << endl;

            disable();
            error_events->notify<Error::Event::tx_error>(id);
        }
        catch (std::system_error& ex) {
            stdout_log << Logging_level::debug;
            stdout_log << "Datagram sender [" << id << "] send() caught exception" << ex.what() << endl;
        
            disable();
            error_events->notify<Error::Event::tx_error>(id);
        }
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Sender<protocol, transport, tls>::enable()
    {
        enabled = true;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Sender<protocol, transport, tls>::disable()
    {
        if (!enabled) return;
        enabled = false;
        
        stdout_log << Logging_level::debug;
        stdout_log << "Datagram sender [" << id << "] disabled" << endl;
    }


    // =================================================================================================================
    //
    template <Protocol protocol, Transport transport, TLS::Type tls>
    class Receiver : public Utility::Active {
    public:
        using Event_traits      = Navtech::Networking::Event_traits<protocol, transport, tls>;
        using Connection_traits = Navtech::Networking::Connection_traits<protocol, transport, tls>;
        using Protocol_traits   = typename Connection_traits::Protocol_traits;
        using Socket_Ty         = typename Connection_traits::Socket;
        using Message_buffer_Ty = typename Connection_traits::Message_buffer;
        using Message_Ty        = typename Connection_traits::Message;
        using ID_Ty             = typename Connection_traits::ID;
        using Dispatcher_Ty     = typename Event_traits::Dispatcher;

        Receiver(
            Socket_Ty&          sckt, 
            ID_Ty               identity, 
            Dispatcher_Ty&      protocol_event_dispatcher,
            Error::Dispatcher&  error_event_dispatcher
        );

        void enable();
        void disable();

    private:
        // External service associations
        //
        association_to<Socket_Ty>           socket;
        association_to<Dispatcher_Ty>       protocol_events;
        association_to<Error::Dispatcher>   error_events;

        // Operating state
        //
        std::atomic<bool> enabled { false };
        ID_Ty   id;

        // Active class overrides
        //
        void on_start() override;
        void on_stop()  override;

        // Buffers for incoming data
        //
        Message_Ty incoming_msg { };

        Active::Task_state run() override;
        void shutdown();
    };


    template <Protocol protocol, Transport transport, TLS::Type tls>
    Receiver<protocol, transport, tls>::Receiver(
        Receiver<protocol, transport, tls>::Socket_Ty&       sckt, 
        Receiver<protocol, transport, tls>::ID_Ty            identity,
        Receiver<protocol, transport, tls>::Dispatcher_Ty&   protocol_event_dispatcher,
        Error::Dispatcher&                                   error_event_dispatcher
    ) :
        Active              { "Receiver [" + std::to_string(identity) + "]" },
        socket              { associate_with(sckt) },
        protocol_events     { associate_with(protocol_event_dispatcher) },
        error_events        { associate_with(error_event_dispatcher) },
        id                  { identity }
    {
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Receiver<protocol, transport, tls>::on_start()
    {
        stdout_log << Logging_level::debug;
        stdout_log << "Datagram receiver [" << id << "] starting..." << endl;

        enable();
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Receiver<protocol, transport, tls>::on_stop()
    {
        stdout_log << Logging_level::debug;
        stdout_log << "Datagram receiver [" << id << "] stopping..." << endl;

        disable();
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    Active::Task_state Receiver<protocol, transport, tls>::run()
    {
        if (!enabled) {
            return Task_state::finished;
        }
        
        if (!socket->is_open()) {
            shutdown();
            return Task_state::finished;
        }

        try {
            auto [_, buffer] = socket->receive();

            Message_Ty incoming_msg { buffer };

            if (Protocol_traits::is_valid(incoming_msg)) {
                Protocol_traits::add_client_id(incoming_msg, id);
                protocol_events->template notify<Event_traits::Received_message>(std::move(incoming_msg));
            }
            
            return Task_state::not_finished;
        }
        catch (client_shutdown&) {
            stdout_log << Logging_level::debug;
            stdout_log << "Datagram receiver [" << id << "] client disconnected" << endl;

            disable();
            shutdown();
            return Task_state::finished;
        }
        catch (std::system_error& ex) {
            stdout_log << Logging_level::debug;
            stdout_log << "Datagram receiver [" << id << "] run() caught exception: " << ex.what() << endl;

            disable();
            shutdown();
            return Task_state::finished;
        }
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Receiver<protocol, transport, tls>::enable()
    {
        enabled = true;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Receiver<protocol, transport, tls>::disable() 
    {
        if (!enabled) return;
        enabled = false;
        
        stdout_log << Logging_level::debug;
        stdout_log << "Datagram receiver [" << id << "] disabled." << endl;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Receiver<protocol, transport, tls>::shutdown()
    {
        stdout_log << Logging_level::debug;
        stdout_log << "Datagram receiver [" << id << "] notify rx error." << endl;

        error_events->notify<Error::Event::rx_error>(id);
    }


    // =================================================================================================================
    //
    template <Protocol protocol, Transport transport, TLS::Type tls>
    class Connection {
    public:
        // Type aliases.
        // The '_Ty' postfix denotes a template type. Rather than being
        // supplied as template parameters on the class (which would be
        // unwieldy) these parameters are looked up from the Connection_traits
        // class, using the appropriate combination of protocol and transport
        //
        using Event_traits      = Navtech::Networking::Event_traits<protocol, transport, tls>;
        using Connection_traits = Navtech::Networking::Connection_traits<protocol, transport, tls>;
        using Protocol_traits   = typename Connection_traits::Protocol_traits;
        using Socket_Ty         = typename Connection_traits::Socket;
        using Message_Ty        = typename Connection_traits::Message;
        using ID_Ty             = typename Connection_traits::ID;
        using Dispatcher_Ty     = typename Event_traits::Dispatcher;

        enum Direction { rx, tx, tx_rx };

        Connection(
            ID_Ty               identifier, 
            Socket_Ty&&         sckt, 
            Dispatcher_Ty&      protocol_event_dispatcher,
            Error::Dispatcher&  error_event_dispatcher,
            Direction           dir = tx
        );
        ~Connection();

        void open();
        void close();

        void bind_to(const Endpoint& local_endpt);
        void remote_endpoint(const Endpoint& remote_endpt);

        void send(const Message_Ty& msg);
        void send(Message_Ty&& msg);

        ID_Ty id() const;

    private:
        // External associations
        //
        association_to<Dispatcher_Ty>       protocol_events;
        association_to<Error::Dispatcher>   error_events;

        // Internal components
        //
        Socket_Ty socket { };
        Sender<protocol, transport, tls>   sender   { socket };
        Receiver<protocol, transport, tls> receiver { socket };

        // Event handling (from sender and receiver)
        //
        Utility::Event_handler<ID_Ty> tx_error_handler { };
        Utility::Event_handler<ID_Ty> rx_error_handler { };
        void on_send_error(const ID_Ty& identity);
        void on_receive_error(const ID_Ty& identity);

        // Internal state
        //
        ID_Ty ident    { };
        bool  enabled  { false };
        Direction dir  { tx };

        bool is_set(Direction d) const
        {
            if (dir == tx_rx)   return true;
            if (dir == d)       return true;
            return false;
        }
    };



    template <Protocol protocol, Transport transport, TLS::Type tls>
    Connection<protocol, transport, tls>::Connection(
        Connection<protocol, transport, tls>::ID_Ty             identity, 
        Connection<protocol, transport, tls>::Socket_Ty&&       sckt,
        Connection<protocol, transport, tls>::Dispatcher_Ty&    protocol_event_dispatcher,
        Error::Dispatcher&                                      error_event_dispatcher,
        Connection<protocol, transport, tls>::Direction         direction
    ) :
        protocol_events { associate_with(protocol_event_dispatcher) },
        error_events    { associate_with(error_event_dispatcher) },
        socket          { std::move(sckt) },
        sender          { socket, identity, protocol_event_dispatcher, error_event_dispatcher },
        receiver        { socket, identity, protocol_event_dispatcher, error_event_dispatcher },
        ident           { identity },
        dir             { direction }
    {   
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    Connection<protocol, transport, tls>::~Connection()
    {
        close();
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection<protocol, transport, tls>::open()
    {
        enabled = true;

        tx_error_handler.when_notified_invoke(&Connection::on_send_error, this);
        rx_error_handler.when_notified_invoke(&Connection::on_receive_error, this);
        error_events->attach_to<Error::Event::tx_error>(tx_error_handler);
        error_events->attach_to<Error::Event::rx_error>(rx_error_handler);

        if (is_set(tx)) sender.start();
        if (is_set(rx)) receiver.start();

        protocol_events->template notify<Event_traits::Client_connected>(ident);

        stdout_log << "Datagram connection [" << ident << "] open." << endl;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection<protocol, transport, tls>::close()
    {
        if (!enabled) return;
        enabled = false;

        error_events->detach_from<Error::Event::tx_error>(tx_error_handler);
        error_events->detach_from<Error::Event::rx_error>(rx_error_handler);

        try {
            socket.close();
        }
        catch (std::system_error&) {
            // We're closing, so ignore any errors
            // the socket may throw.
        }

        if (is_set(tx)) {
            sender.stop();
        }

        if (is_set(rx)) {
            receiver.stop();
            receiver.join();
        }

        stdout_log << "Datagram connection [" << ident << "] closed. "
                   << "Sent ["     << Utility::to_memory_string(socket.bytes_sent()) << "] "
                   << "Received [" << Utility::to_memory_string(socket.bytes_read()) << "] "
                   << endl;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection<protocol, transport, tls>::on_send_error(const ID_Ty& session_in_error)
    {
        if (!enabled)                     return;
        if (session_in_error != this->ident) return;

        // The Sender will have already disabled itself
        // so disable the Receiver explicitly
        //
        receiver.disable();

        // The receiver of the connection_error event 
        // should terminate this connection.
        //
        stdout_log << Logging_level::debug;
        stdout_log << "Datagram connection [" << ident << "] send error.  Requesting termination" << endl;

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
        stdout_log << "Datagram connection [" << ident << "] receiver error.  Requesting termination" << endl;

        error_events->notify<Error::Event::connection_error>(ident);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection<protocol, transport, tls>::bind_to(const Endpoint& local_endpt)
    {
        socket.bind_to(local_endpt);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection<protocol, transport, tls>::remote_endpoint(const Endpoint& remote_endpt)
    {
        socket.remote_endpoint(remote_endpt);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection<protocol, transport, tls>::send(const Connection<protocol, transport, tls>::Message_Ty& msg)
    {
        if (!enabled) return;

        sender.send(Protocol_traits::to_buffer(msg));
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection<protocol, transport, tls>::send(Connection<protocol, transport, tls>::Message_Ty&& msg)
    {
        if (!enabled) return;

        sender.send(Protocol_traits::to_buffer(std::move(msg)));
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    typename Connection<protocol, transport, tls>::ID_Ty 
    Connection<protocol, transport, tls>::id() const
    {
        return ident;
    }


} // Navtech::Networking::Datagram

#endif // DATAGRAM_CONNECTION_H