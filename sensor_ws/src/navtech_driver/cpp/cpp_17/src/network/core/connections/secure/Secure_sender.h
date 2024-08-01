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
#ifndef SECURE_SENDER_H
#define SECURE_SENDER_H

#include "Connection_traits.h"
#include "Port.h"
#include "Log.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;


namespace Navtech::Networking::Secure {

    template <Protocol protocol, Transport transport = Transport::tcp, TLS::Type tls = TLS::Type::none>
    class Sender {
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
        using Message_buffer_Ty = typename Connection_traits::Message_buffer;
        using ID_Ty             = typename Connection_traits::ID;
        using Dispatcher_Ty     = typename Event_traits::Dispatcher;
        using In_port           = Utility::Port<Message_buffer_Ty>;

        Sender( 
            Socket_Ty&          sckt, 
            ID_Ty               id, 
            Dispatcher_Ty&      event_dispatcher,
            Error::Dispatcher&  error_event_dispatcher
        );

        In_port in { };

        void start();
        void stop();

        void send(const Message_buffer_Ty& msg);
        void send(Message_buffer_Ty&& msg);

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
    };



    template <Protocol protocol, Transport transport, TLS::Type tls>
    Sender<protocol, transport, tls>::Sender(
        Sender<protocol, transport, tls>::Socket_Ty&       sckt, 
        Sender<protocol, transport, tls>::ID_Ty            idity, 
        Sender<protocol, transport, tls>::Dispatcher_Ty&   event_dispatcher,
        Error::Dispatcher&                                 error_event_dispatcher
    ) :
        socket              { associate_with(sckt) },
        protocol_events     { associate_with(event_dispatcher) },
        error_events        { associate_with(error_event_dispatcher) },
        id                  { idity }
    {
        in.on_receive([this](Message_buffer_Ty& buf) { send(std::move(buf)); });
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Sender<protocol, transport, tls>::start()
    {
        stdout_log << Logging_level::debug;
        stdout_log << "Secure sender [" << id << "] starting..." << endl;

        enable();
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Sender<protocol, transport, tls>::stop()
    {
        stdout_log << Logging_level::debug;
        stdout_log << "Secure sender [" << id << "] stopping..." << endl;

        disable();
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Sender<protocol, transport, tls>::send(const Sender<protocol, transport, tls>::Message_buffer_Ty& msg)
    {
        if (!enabled) return;

        try {
            socket->send(msg);
        }
        catch (std::system_error& ex) {
            stdout_log << Logging_level::debug;
            stdout_log << "Secure sender [" << id << "] caught exception: " << ex.what() << endl;
 
            disable();
            error_events->notify<Error::Event::tx_error>(id);
        }
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Sender<protocol, transport, tls>::send(Sender<protocol, transport, tls>::Message_buffer_Ty&& msg)
    {
        if (!enabled) return;

        try {
            socket->send(std::move(msg));
        }
        catch (std::system_error& ex) {
            stdout_log << Logging_level::debug;
            stdout_log << "Secure sender [" << id << "] caught exception: " << ex.what() << endl;

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
        in.disconnect();

        stdout_log << Logging_level::debug;
        stdout_log << "Secure sender [" << id << "] disabled " << endl;
    }


} // namespace Navtech::Networking::Secure


#endif // SECURE_SENDER_H