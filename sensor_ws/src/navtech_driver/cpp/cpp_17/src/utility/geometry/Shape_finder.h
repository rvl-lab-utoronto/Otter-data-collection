#ifndef SHAPE_FINDER_H
#define SHAPE_FINDER_H
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

#include <algorithm>
#include <deque>
#include <unordered_set>
#include <utility>
#include <vector>

#include "float_equality.h"

namespace Navtech::Utility {

template<typename T>
class Shape_finder {
public:
    Shape_finder(int minimum_col);

    std::vector<std::pair<float, float>>    find_centres(const std::vector<std::vector<T>>& data);
private:
    // Memoisation and hashing
    //
    struct Pair_hash {
        std::size_t operator()(const std::pair<int, int>& p) const {
            auto hash1 = std::hash<int>{}(p.first);
            auto hash2 = std::hash<int>{}(p.second);
            return hash1 ^ hash2;
        }
    };

    std::unordered_set<std::pair<int, int>, Pair_hash>     explored_cells   { };

    // Depth-first search directions
    //
    static constexpr std::array<std::array<int, 2>, 4>  directions { {{0, 1}, {1, 0}, {0, -1}, {-1, 0}} };

    // Cells to test next
    //
    std::deque<std::pair<int, int>>     test_cells  { };
    
    int     min_col { 0 };
    int     rows    { 0 };
    int     cols    { 0 };
    
    int    wrap_row(int x);
    
    std::optional<std::pair<float, float>>     depth_first_search(const std::vector<std::vector<T>>& data);
};

template<typename T>
Shape_finder<T>::Shape_finder(int minimum_col) : min_col {minimum_col}
{

}


template<typename T>
std::vector<std::pair<float, float>> Shape_finder<T>::find_centres(const std::vector<std::vector<T>>& data)
{
    std::vector<std::pair<float, float>>    output_pairs    { };
    rows = static_cast<int>(data.size());
    cols = static_cast<int>(data[0].size());

    for (auto i { 0 }; i < rows; ++i) {
        for (auto j { 0 }; j < cols; ++j) {
            if (j < min_col || explored_cells.count({ i, j }) || essentially_equal(data[i][j], 0))
                continue;

            test_cells.push_back({i, j});

            auto shape_centre = depth_first_search(data);
            
            if (!shape_centre.has_value()) continue;

            output_pairs.push_back(shape_centre.value());
        }
    }
    return output_pairs;
}


template<typename T>
int Shape_finder<T>::wrap_row(int x)
{
    auto wrapped { x };
    if (wrapped < 0) {
        wrapped = rows + x;
    }
    else if (wrapped >= rows) {
        wrapped = x - rows;
    }

    return wrapped;
}


template<typename T>
std::optional<std::pair<float, float>> Shape_finder<T>::depth_first_search(const std::vector<std::vector<T>>& data)
{
    using Utility::essentially_equal;

    auto    shape_size      { 0 };
    float   total_mass      { 0.0f };
    float   row_moment_sum  { 0.0f };
    float   col_moment_sum  { 0.0f };

    while (!test_cells.empty()) {
        auto [x, y] = test_cells.front();
        test_cells.pop_front();

        auto x2 = wrap_row(x);

        if (
            data[x2].empty() || essentially_equal(data[x2][y], 0.0f) ||
            explored_cells.count({x2, y}) || std::isnan(data[x2][y])
        ) {
            continue;
        }

        shape_size++;
        float current_mass = static_cast<float>(data[x2][y]);
        total_mass += current_mass;
        col_moment_sum += current_mass * y;
        row_moment_sum += current_mass * x;

        explored_cells.insert({x2, y});

        for (auto [dx, dy] : directions) {
            int nx = x + dx;
            int nx2 = wrap_row(nx);

            int ny = y + dy;

            if (ny >= 0 && ny < cols && !explored_cells.count({nx2, ny})) {
                test_cells.push_back({nx, ny});
            }
        }
    }

    if (shape_size == 1) return { };

    return std::make_pair<float, float>(row_moment_sum / total_mass, col_moment_sum / total_mass);
}
} // namespace Navtech::Utility
#endif