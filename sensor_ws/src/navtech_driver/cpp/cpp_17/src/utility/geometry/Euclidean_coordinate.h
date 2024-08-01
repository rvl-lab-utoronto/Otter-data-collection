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
#ifndef EUCLIDEAN_COORDINATE_H
#define EUCLIDEAN_COORDINATE_H

#include "Units.h"

namespace Navtech::Spherical { struct Coordinate; }

namespace Navtech::Euclidean {

    // +ve Y is along radar North
    // +ve X is along radar East
    // 
    struct Coordinate {
        Unit::Metre x { };
        Unit::Metre y { };
        Unit::Metre z { };


        Spherical::Coordinate to_spherical() const;

        // Comparison
        //
        bool operator==(const Coordinate& rhs) const;
        bool operator!=(const Coordinate& rhs) const;

        // Operations
        //
        Coordinate operator+(const Coordinate& rhs) const;
        Coordinate& operator+=(const Coordinate& rhs);

        // Linear distance between two coordinates
        //
        Unit::Metre operator-(const Coordinate& rhs) const;

        std::string to_string() const;
    };

} // namespace Navtech::Cartesian

#endif // EUCLIDEAN_COORDINATE