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
#ifndef SECURE_RECEIVER_H
#define SECURE_RECEIVER_H

#include "Connection_traits.h"
#include "Port.h"
#include "Log.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;

namespace Navtech::Networking::Secure {

    template <Protocol protocol, Transport transport = Transport::tcp, TLS::Type tls = TLS::Type::none>
    class Receiver : public Utility::Active {
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
        using Message_Ty        = typename Connection_traits::Message;
        using ID_Ty             = typename Connection_traits::ID;
        using Dispatcher_Ty     = typename Event_traits::Dispatcher;
        using Out_port          = Utility::Port<Message_buffer_Ty>;

        Receiver(
            Socket_Ty&          sckt, 
            ID_Ty               identity, 
            Dispatcher_Ty&      protocol_event_dispatcher,
            Error::Dispatcher&  error_event_dispatcher
        );

        void enable();
        void disable();

        Out_port out { };

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
        void dispatch();
        void shutdown();
    };


    template <Protocol protocol, Transport transport, TLS::Type tls>
    Receiver<protocol, transport, tls>::Receiver(
        Receiver<protocol, transport, tls>::Socket_Ty&       sckt, 
        Receiver<protocol, transport, tls>::ID_Ty            identity,
        Receiver<protocol, transport, tls>::Dispatcher_Ty&   protocol_event_dispatcher,
        Error::Dispatcher&                                   error_event_dispatcher
    ) :
        Active              { "Secure receiver [" + std::to_string(identity) + "]" },
        socket              { associate_with(sckt) },
        protocol_events     { associate_with(protocol_event_dispatcher) },
        error_events        { associate_with(error_event_dispatcher) },
        id                  { identity }
    {
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Receiver<protocol, transport, tls>::on_start()
    {
        enabled = true;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Receiver<protocol, transport, tls>::on_stop()
    {
        out.disconnect();
        enabled = false;
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
            auto payload = socket->receive_some(Connection_traits::max_buffer_size);
            out.post(std::move(payload));

            return Task_state::not_finished;
        }
        catch (client_shutdown&) {
            stdout_log << Logging_level::debug;
            stdout_log << "Secure receiver [" << id << "] client disconnected." << endl;
            
            disable();
            shutdown();
            return Task_state::finished;
        }
        catch (std::system_error& e) {
            stdout_log << Logging_level::debug;
            stdout_log << "Secure receiver [" << id << "] caught exception: " << e.what() << endl;

            disable();
            shutdown();
            return Task_state::finished;
        }
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Receiver<protocol, transport, tls>::shutdown()
    {
        stdout_log << Logging_level::debug;
        stdout_log << "Secure receiver [" << id << "] shutdown.  Requesting termination." << endl;

        error_events->notify<Error::Event::rx_error>(id);
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

        stdout_log << "Secure receiver [" << id << "] disabled." << endl;
    }


} // namespace Navtech::Networking::Secure

#endif // SECURE_RECEIVER_H