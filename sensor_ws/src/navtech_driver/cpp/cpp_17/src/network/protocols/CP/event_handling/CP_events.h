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
#ifndef CP_EVENTS_H
#define CP_EVENTS_H

#include "Event_dispatcher.h"
#include "CP_protocol.h"


namespace Navtech::Networking::CP_protocol {

    enum class Event {
        send_message,           // A message is ready to be sent
        received_message,       // A message has been received
        client_connected,       // A new client connection has been established
        client_disconnected     // A client connection has disconnected

        // More, as required...
	};  

    // See Event_dispatcher.h for details on how to configure Event_traits
    //
	template <Event> struct Event_traits 				            { using Parameter = Message::ID; };
    template <> struct Event_traits<Event::send_message>            { using Parameter = Message; };
    template <> struct Event_traits<Event::received_message>        { using Parameter = Message; };
	
	using Event_dispatcher = Utility::Dispatcher<Event, Event_traits>;

} // namespace Navtech::Networking::Colossus_protocol

#endif // CP_EVENTS_H