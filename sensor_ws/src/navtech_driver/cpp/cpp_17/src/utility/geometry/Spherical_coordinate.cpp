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
#include <cmath>
#include "Spherical_coordinate.h"
#include "Euclidean_coordinate.h"
#include "float_equality.h"

namespace Navtech::Spherical {

    Euclidean::Coordinate Coordinate::to_euclidean() const
    {
        return Euclidean::Coordinate {
            std::round((range * std::sin(bearing.to_radians()) * std::sin(theta.to_radians())) * 1000.0f) / 1000.0f,
            std::round((range * std::cos(bearing.to_radians()) * std::sin(theta.to_radians())) * 1000.0f) / 1000.0f,
            std::round((range * std::cos(theta.to_radians())) * 1000.0f) / 1000.0f
        };
    }


    bool Coordinate::operator==(const Coordinate& rhs) const
    {
        // Spherical coordinates are considered equal if:
        // - their ranges are within 1mm of each other at 100m
        // - their bearings are within 0.001 degrees
        // - their thetas are within 0.001 degrees
        //
        return  (Utility::essentially_equal(this->range, rhs.range, 0.00001f)) && 
        (this->bearing == rhs.bearing) && (this->theta == rhs.theta);
    }

    bool Coordinate::operator!=(const Coordinate& rhs) const
    {
        return !(*this == rhs);
    }


    std::string Coordinate::to_string() const
    {
        std::stringstream stream { };

        stream << "[";
        stream << std::fixed << std::setprecision(2);
        stream << bearing.to_string() << ", ";
        stream << range << "m" << ", ";
        stream << theta.to_string();
        stream << "]";

        return stream.str();
    }
} // namespace Navtech::Spherical