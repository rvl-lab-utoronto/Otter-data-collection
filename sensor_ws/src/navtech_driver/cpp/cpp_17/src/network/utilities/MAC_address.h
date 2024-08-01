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
#ifndef MAC_ADDRESS_H
#define MAC_ADDRESS_H

#include <cstdint>
#include <string_view>
#include <array>

#include "Endian.h"

namespace Navtech {

    namespace Networking {

        class MAC_address {
        public:
            static constexpr std::size_t num_octets { 6 };
            using Byte_array = std::array<std::uint8_t, num_octets>;
            
            MAC_address() = default;
            MAC_address(std::string_view add_str);
            MAC_address(const std::uint8_t* const byte_array);
            MAC_address(const Byte_array& byte_array);

            MAC_address& operator=(const Byte_array& mac_addr);
            MAC_address& operator=(std::string_view mac_str);
        
            Byte_array   to_byte_array() const;
            std::string  to_string() const;

        private:
            std::array<std::uint8_t, num_octets> octets { };

            Byte_array from_string(std::string_view add_str);
        };


    } // namespace Networking


    // User-defined literals
    //
    inline Networking::MAC_address operator""_mac(const char* mac_str, std::size_t sz)
    {
        return Networking::MAC_address { std::string_view { mac_str, sz } };
    }


} // namespace Navtech

#endif // MAC_ADDRESS_H