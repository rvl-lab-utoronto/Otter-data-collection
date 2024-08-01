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
#ifndef SECURE_MARSHALLER_H
#define SECURE_MARSHALLER_H

#include "Connection_traits.h"
#include "Event_traits.h"
#include "Port.h"

namespace Navtech::Networking::Secure {

    template <Protocol protocol, Transport transport = Transport::tcp, TLS::Type tls = TLS::Type::none>
    class Marshaller {
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
        using Message_Ty        = typename Connection_traits::Message;
        using Message_buffer_Ty = typename Connection_traits::Message_buffer;
        using ID_Ty             = typename Connection_traits::ID;
        using Dispatcher_Ty     = typename Event_traits::Dispatcher;
        using In_port           = Utility::Port<Message_Ty>;
        using Out_port          = Utility::Port<Message_buffer_Ty>;

        In_port  in  { };
        Out_port out { };

        Marshaller();

        void start();
        void stop();

        void send(const Message_Ty& msg);
        void send(Message_Ty&& msg);

    private:
    };


    template <Protocol protocol, Transport transport, TLS::Type tls>
    Marshaller<protocol, transport, tls>::Marshaller()
    {
        in.on_receive(
            [this](Message_Ty& msg) 
            { 
                send(std::move(msg));
            }
        );
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Marshaller<protocol, transport, tls>::start()
    {
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Marshaller<protocol, transport, tls>::stop()
    {
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Marshaller<protocol, transport, tls>::send(const Message_Ty& msg)
    {
        out.post(Protocol_traits::to_buffer(msg));
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Marshaller<protocol, transport, tls>::send(Message_Ty&& msg)
    {
        out.post(Protocol_traits::to_buffer(msg));
    }


} // namespace Navtech::Networking::Secure


#endif // SECURE_MARSHALLER_H