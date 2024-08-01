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
#ifndef NETWORK_PORT_H
#define NETWORK_PORT_H

#include <cstdint>
#include <string>

#include "net_conversion.h"

namespace Navtech::Networking {

    class Port {
    public:
        Port() = default;
        Port(std::uint16_t port_num) : port { port_num } { }
        Port(std::uint16_t port_num, Endian from) : port { from == Endian::host ? port_num : to_uint16_host(port_num) } { }
        Port(const std::string& str) : port { static_cast<std::uint16_t>(std::stoi(str)) } { }

        std::uint16_t to_uint16() const              { return port; }
        std::uint16_t to_uint16(Endian to) const     { return (to == Endian::host ? port : to_uint16_network(port)); }
        std::string   to_string() const              { return std::to_string(port); }

        bool operator==(const Port& rhs) const       { return this->port == rhs.port; }
        bool operator!=(const Port& rhs) const       { return !(*this == rhs); }
    
    private:
        std::uint16_t port { };
    };

} // namepsace Navtech::Networking

#endif // NETWORK_PORT_H