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
#ifndef CRC_GENERATOR_H
#define CRC_GENERATOR_H

#include <cstdint>
#include <array>
#include <algorithm>
#include <numeric>

namespace Navtech::Utility {

    // Generates a lookup table for the checksums of all 8-bit values.
    //
    inline std::array<std::uint_fast32_t, 256> generate_crc_lookup_table() noexcept
    {
        auto const reversed_polynomial = std::uint_fast32_t { 0xEDB88320uL };
         
        // This is a function object that calculates the checksum for a value,
        // then increments the value, starting from zero.
        //
        struct byte_checksum
        {
            std::uint_fast32_t operator()() noexcept
            {
                auto checksum = static_cast<std::uint_fast32_t>(n++);
                
                for (auto i = 0; i < 8; ++i) {
                    checksum = (checksum >> 1) ^ ((checksum & 0x1u) ? reversed_polynomial : 0);
                }
                
                return checksum;
            }
            
            unsigned n { 0 };
        };
        
        auto table = std::array<std::uint_fast32_t, 256>{};
        std::generate(table.begin(), table.end(), byte_checksum{});
        
        return table;
    }
    

    // Calculates the CRC for any sequence of values. (You could use type traits and a
    // static assert to ensure the values can be converted to 8 bits.)
    //
    template <typename InputIterator>
    std::uint_fast32_t CRC(InputIterator first, InputIterator last)
    {
        // Generate lookup table only on first use 
        // then cache it - this is thread-safe.
        //
        static auto const table = generate_crc_lookup_table();
        
        // Calculate the checksum - make sure to clip to 32 bits, 
        // for systems that don't have a true (fast) 32-bit type.
        //
        return std::uint_fast32_t{ 0xFFFFFFFFuL } &
            ~std::accumulate(
                first, 
                last,
                ~std::uint_fast32_t { 0 } & std::uint_fast32_t{0xFFFFFFFFuL},
                [](std::uint_fast32_t checksum, std::uint_fast8_t value) 
                { 
                    return table[(checksum ^ value) & 0xFFu] ^ (checksum >> 8); 
                }
            );
    }


} // namespace Navtech::Utility

#endif // CRC_GENERATOR_H
