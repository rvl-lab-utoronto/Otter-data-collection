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
#include "Spherical_target.h"
#include "Euclidean_target.h"
#include "float_equality.h"

namespace Navtech::Navigation::Spherical {

    Navigation::Euclidean::Target Target::to_euclidean() const
    {
        auto euclidean_coord = coordinate.to_euclidean();
        return { euclidean_coord.x, euclidean_coord.y, euclidean_coord.z, power };
    }


    bool Target::operator==(const Target& rhs) const
    {
        return  this->coordinate == rhs.coordinate &&
                (Utility::essentially_equal(this->power, rhs.power, 0.00001f));
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
        stream << coordinate.range << ", " << coordinate.bearing << ", " << coordinate.theta;
        stream << ", " << power << "]";

        return stream.str();
    }
}