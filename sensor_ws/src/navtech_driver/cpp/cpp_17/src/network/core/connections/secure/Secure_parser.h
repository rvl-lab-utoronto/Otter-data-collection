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
#ifndef SECURE_PARSER_H
#define SECURE_PARSER_H

#include "Connection_traits.h"
#include "Event_traits.h"
#include "Port.h"
#include "Circular_buffer.h"
#include "Log.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;

namespace Navtech::Networking::Secure {

    template <Protocol protocol, Transport transport = Transport::tcp, TLS::Type tls = TLS::Type::none>
    class Parser {
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
        using Iterator_Ty       = typename Protocol_traits::Iterator;
        using In_Port           = Utility::Port<Message_buffer_Ty>;

        In_Port in { };

        Parser(ID_Ty id, Dispatcher_Ty& event_dispatcher);

        void start();
        void stop();

    private:
        // External associations
        //
        association_to<Dispatcher_Ty>   dispatcher;

        ID_Ty id;

        // Message parsing
        //
        void process(const Message_buffer_Ty& msg);
        void process(Message_buffer_Ty&& msg);
        void parse_input_stream();

        using Input_stream = Utility::Circular_buffer<std::uint8_t, 16_kB>;

        Input_stream        input_stream         {  };
        Message_Ty          message              {  };
        Message_buffer_Ty   header_buffer        {  };
        Message_buffer_Ty   payload_buffer       {  };
        std::size_t         current_header_size  {  };
        std::size_t         current_payload_size {  };
        Iterator_Ty         write_pos            {  };

        // State machine for controlling parsing
        //
        enum State { reading_header, reading_payload, num_states };
        State current_state { reading_header };

        using State_behaviour = State(Parser::*)();
        State do_read_header();
        State do_read_payload();
        void  on_header_received();
        void  on_message_complete();
        void  initialize_message();

        static std::array<State_behaviour, State::num_states> state_table;
    };


    template <Protocol protocol, Transport transport, TLS::Type tls>
    Parser<protocol, transport, tls>::Parser(
        Parser<protocol, transport, tls>::ID_Ty          identity,
        Parser<protocol, transport, tls>::Dispatcher_Ty& event_dispatcher
    ) :
        dispatcher    { associate_with(event_dispatcher) },
        id            { identity }
    {
        in.on_receive([this](Message_buffer_Ty& msg) { process(std::move(msg)); } );
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Parser<protocol, transport, tls>::start()
    {
        initialize_message();
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Parser<protocol, transport, tls>::stop()
    { 
        // TODO - clear out input stream?
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Parser<protocol, transport, tls>::process(const Message_buffer_Ty& msg)
    {
        input_stream.push(msg);
        parse_input_stream();
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Parser<protocol, transport, tls>::process(Message_buffer_Ty&& msg)
    {
        using std::move;

        input_stream.push<std::vector>(move(msg));
        parse_input_stream();
    }


    // This function attempts to construct a complete and valid message from the input 
    // stream. The algorithm logic is as follows:
    // 
    // - If there's not enough data in the input stream to construct a complete header, 
    //   wait until more data is pushed onto the input stream.
    //
    // - Read a header's worth of data and see if it is a valid message header.  If 
    //   not, discard this data and try again. Repeat until we have a valid header, 
    //   or run out of data on the input stream.
    //
    // - Read a payload's worth of data from the input stream. If there's not (yet) 
    //   enough data on the input stream to read a full payload, retrieve what is 
    //   available, then wait for the next insertion on the input stream.  Repeat
    //   until a full payload has been retrieved.
    //
    // - Output the message.
    //
    // - Repeat while there's still data on the input stream
    //
    // This algorithm is implemented as a simple state machine.  The 'do-' functions 
    // represent the in-state behaviour; the 'on-' functions represent the transition 
    // behaviours.
    //
    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Parser<protocol, transport, tls>::parse_input_stream()
    {
        while (!input_stream.empty()) {
            current_state = (this->*state_table[current_state])();
        }
    }


    // Parser state machine implementation
    //
    template <Protocol protocol, Transport transport, TLS::Type tls>
    std::array<typename Parser<protocol, transport, tls>::State_behaviour, Parser<protocol, transport, tls>::State::num_states> 
    Parser<protocol, transport, tls>::state_table {
        &Parser::do_read_header,
        &Parser::do_read_payload
    };


    // - Read a header's worth of data and see if it is a valid
    //   message header.  If there's not enough data in the input 
    //   stream to construct a complete header, wait until more data 
    //   is pushed onto the input stream.
    //   Repeat until we have a valid header.
    //   If the message has a payload configure the parser to read
    //   it; otherwise forward on the (completed) message.
    //
    template <Protocol protocol, Transport transport, TLS::Type tls>
    typename Parser<protocol, transport, tls>::State Parser<protocol, transport, tls>::do_read_header()
    {
        using std::move;

        auto num_bytes = input_stream.pop_n_into(
            write_pos, 
            (Protocol_traits::header_size(message) - current_header_size)
        );

        current_header_size += num_bytes;
        write_pos           += num_bytes;

        if (current_header_size < Protocol_traits::header_size(message)) {
            return reading_header;
        }
        else {
            Protocol_traits::add_header(message, move(header_buffer));
        }

        if (!Protocol_traits::is_valid(message)) {
            stdout_log << "Parser [" << id << "] invalid message" << endl;
            
            initialize_message();
            return reading_header;
        }

        if (Protocol_traits::payload_size(message) > 0) {
            on_header_received();
            return reading_payload;
        }
        else {
            on_message_complete();
            return reading_header;
        }  
    }


    // Set up the current message prior to receiving
    // its payload.
    //
    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Parser<protocol, transport, tls>::on_header_received()
    {
        using std::begin;

        // Set up the payload buffer ready to accept data
        //
        payload_buffer.resize(Protocol_traits::payload_size(message));
        current_payload_size = 0;
        write_pos = payload_buffer.data();
    }


    // - Read a payload's worth of data from the input stream.
    //   If there's not (yet) enough data on the input stream to read
    //   a full payload, retrieve what is available, then wait for the
    //   next insertion on the input stream.  Repeat until a full
    //   payload has been retrieved.
    //
    template <Protocol protocol, Transport transport, TLS::Type tls>
    typename Parser<protocol, transport, tls>::State Parser<protocol, transport, tls>::do_read_payload()
    {
        using std::move;
        using std::begin;
        using std::end;

        auto num_bytes = input_stream.pop_n_into(
            write_pos, 
            (Protocol_traits::payload_size(message) - current_payload_size)
        );

        current_payload_size += num_bytes;
        write_pos            += num_bytes;

        if (current_payload_size != Protocol_traits::payload_size(message)) {
            return reading_payload;
        }

        Protocol_traits::add_payload(message, move(payload_buffer));

        on_message_complete();
        return reading_header;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Parser<protocol, transport, tls>::on_message_complete()
    {
        using std::move;
    
        Protocol_traits::add_client_id(message, id);

        dispatcher->template notify<Event_traits::Received_message>(std::move(message));

        initialize_message();
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Parser<protocol, transport, tls>::initialize_message()
    {
        Protocol_traits::clear(message);
        header_buffer.resize(Protocol_traits::header_size(message));
        payload_buffer.clear();
        write_pos            = header_buffer.data();
        current_header_size  = 0;
        current_payload_size = 0;
    }

} // namespace Navtech::Networking::Secure

#endif // SECURE_PARSER_H