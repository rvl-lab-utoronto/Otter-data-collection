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
#include "Euclidean_target.h"
#include "Spherical_target.h"
#include "float_equality.h"

namespace Navtech::Navigation::Euclidean {

    Navigation::Spherical::Target Target::to_spherical() const
    {
        auto spherical_coord = coordinate.to_spherical();

        return { spherical_coord.range, spherical_coord.bearing, spherical_coord.theta, power };
    }


    bool Target::operator==(const Target& rhs) const
    {
        // Coordinate equal, powers must be withing 0.1 dB 
        // (dB is usually in half-integer steps)
        //
        return  this->coordinate == rhs.coordinate &&
                (Utility::essentially_equal(this->power, rhs.power, 0.1f));
    }


    bool Target::operator!=(const Target& rhs) const
    {
        return !(*this == rhs);
    }


    std::string Target::to_string() const
    {
        std::stringstream   stream    { };

        stream << "[";
        stream << std::fixed << std::setprecision(2);
        stream << coordinate.x << ", " << coordinate.y << ", " << coordinate.z;
        stream << ", " << power << "]";

        return stream.str();
    }
} // namespace Navtech::Navigation