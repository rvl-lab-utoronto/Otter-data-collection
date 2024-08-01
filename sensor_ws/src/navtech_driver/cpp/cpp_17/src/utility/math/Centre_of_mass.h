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

#ifndef FEATURE_EXTRACTION_H
#define FEATURE_EXTRACTION_H

#include <vector>
#include <algorithm>
#include <cmath>
#include <numeric>

#include "Units.h"
#include "Log.h"
#include "Vector_maths.h"

namespace Navtech::Utility {

    template <typename Iterator_Ty>
    auto centre_of_mass(Iterator_Ty first, Iterator_Ty last) {
        using val_type = typename std::iterator_traits<Iterator_Ty>::value_type;
        // Distances as bins from starting bin
        //
        float mass_sum          { std::accumulate(first, last, static_cast<val_type>(0)) };        
        float sum_of_moments    { 0 };

        auto sz = std::distance(first, last);
        for ( int i { 0 }; i < sz; ++i) {
            sum_of_moments += (*first) * i;
        }
        
        return sum_of_moments / mass_sum;
    }


    template <typename T>
    auto centre_of_mass(const std::vector<T> power_data) {
        return centre_of_mass(power_data.begin(), power_data.end());
    }

    
    template <typename T>
    std::pair<float, float> centre_of_mass(const std::vector<std::vector<T>>& power_data)
    {
        float total_mass { 0 };
        float mass_azimuth_sum { 0 };
        float mass_bin_sum { 0 };

        for (std::size_t i { 0 }; i < power_data.size(); ++i) {
            float azimuth_mass { 0 };
            float dist = 1 + i;
            
            azimuth_mass = std::accumulate(power_data[i].begin(), power_data[i].end(), 0);

            auto az_centre = centre_of_mass(power_data[i]);

            total_mass += azimuth_mass;
            mass_bin_sum += azimuth_mass * az_centre;
            mass_azimuth_sum += azimuth_mass * dist;
        }

        return std::make_pair<float, float>(
            mass_bin_sum / total_mass,
            mass_azimuth_sum / total_mass
        );
    }
}
#endif 