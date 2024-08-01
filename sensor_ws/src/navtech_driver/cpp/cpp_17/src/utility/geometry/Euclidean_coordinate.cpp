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
#include "Euclidean_coordinate.h"
#include "Spherical_coordinate.h"
#include "float_equality.h"

namespace Navtech::Euclidean {

    Spherical::Coordinate Coordinate::to_spherical() const
    {
        using namespace std;
        using namespace Unit;

        Metre   range   { };
        Radians bearing { };
        Radians theta   { };

        range = hypot(x, y, z);
        
        if (y >= 0.0f) bearing = Radians { atan(x / y) };
        if (y < 0.0f)  bearing = Radians { atan(x / y) + pi<float> };

        theta = std::acos( z / range );

        return Spherical::Coordinate {
            range,
            bearing.to_degrees(),
            theta.to_degrees()
        };
    }


    bool Coordinate::operator==(const Coordinate& rhs) const
    {
        // Cartesian coordinates are considered equal if they
        // are within 1mm of each other at 100m.  This is well below
        // the resolution of the radar.
        //
        return  (Utility::essentially_equal(this->x, rhs.x, 0.00001f)) && 
                (Utility::essentially_equal(this->y, rhs.y, 0.00001f)) &&
                (Utility::essentially_equal(this->z, rhs.z, 0.00001f));
    }


    bool Coordinate::operator!=(const Coordinate& rhs) const
    {
        return !(*this == rhs);
    }


    Coordinate Coordinate::operator+(const Coordinate& rhs) const
    {
        return Coordinate { 
            this->x + rhs.x,
            this->y + rhs.y,
            this->z + rhs.z
        };
    }


    Coordinate& Coordinate::operator+=(const Coordinate& rhs)
    {
        this->x += rhs.x;
        this->y += rhs.y;
        this->z += rhs.z;

        return *this;
    }


    Unit::Metre Coordinate::operator-(const Coordinate& rhs) const
    {
        return std::hypot(
            this->x - rhs.x,
            this->y - rhs.y,
            this->z - rhs.z
        );
    }


    std::string Coordinate::to_string() const
    {
        std::stringstream stream { };

        stream << "[";
        stream << std::fixed << std::setprecision(2);
        stream << x << ", " << y << ", " << z;
        stream << "]";

        return stream.str();
    }
}