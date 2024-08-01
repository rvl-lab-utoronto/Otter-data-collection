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
#ifndef CFAR_ALGORITHMS_H
#define CFAR_ALGORITHMS_H

#include <cstdint>
#include <vector>
#include <array>
#include <numeric>
#include <algorithm>

#include "Units.h"


namespace Navtech::CFAR {
    
    // -----------------------------------------------------------------------------------------------------------------
    // Point
    //
    struct Point {
        Unit::Metre range;
        Unit::dB    power;

        Point(Unit::Metre rng, Unit::dB pwr) : range { rng }, power { pwr }
        {
        }
    };

    // -----------------------------------------------------------------------------------------------------------------
    // Traits.
    // When adding, for example, std::uint8_t values, overflow is a real possibility,
    // therefore use a type that can hold the largest sum.
    //
    template <typename T> 
    class Window_traits {
    public:
        using Sum_Ty = T;

        static T from_dB(Unit::dB power)
        {
            return static_cast<T>(power);
        }

        static Unit::dB to_dB(T power)
        {
            return static_cast<Unit::dB>(power);
        }
    };


    template <>
    class Window_traits<std::uint8_t> {
    public:
        using Sum_Ty = std::uint32_t;

        static std::uint8_t from_dB(Unit::dB power)
        {
            return static_cast<std::uint8_t>(power * 2);
        }

        static Unit::dB to_dB(std::uint8_t power)
        {
            return static_cast<Unit::dB>(power) / 2.0f;
        }
    };


    template <>
    class Window_traits<std::uint16_t> {
    public:
        using Sum_Ty = std::uint32_t;

        static std::uint16_t from_dB(Unit::dB power)
        {
            return static_cast<std::uint16_t>(power * 2);
        }

        static Unit::dB to_dB(std::uint16_t power)
        {
            return static_cast<Unit::dB>(power) / 2.0f;
        }
    };


    // -----------------------------------------------------------------------------------------------------------------
    //
    template <typename T>
    class Window {
    public:
        Window() = default;
        Window(
            const T*    min_bin,
            const T*    max_bin,
            Unit::Bin   window_size,
            Unit::Bin   guard_cells,
            Unit::dB    threshold_delta
        );

        std::size_t size() const;

        std::pair<Unit::dB, std::size_t> process_cell(const T* elem_ptr);
       
    private:
        using Traits = Window_traits<T>;
        using Sum_Ty = typename Traits::Sum_Ty;

        Unit::dB    delta       { };
        Unit::Bin   window_sz   { };
        Unit::Bin   guard_sz    { };
        const T*    range_start { };
        const T*    range_end   { };

        std::size_t threshold_exceeded_count { };

        static constexpr Unit::Bin min_training_cells { 1 };
        
        constexpr Unit::Bin force_odd(Unit::Bin sz) const
        {
            return (sz % 2) != 0 ? sz : sz + 1;
        }

        constexpr Unit::Bin resize_window(Unit::Bin sz) 
        {
            Unit::Bin min_window_sz = (2 * guard_sz) + (2 * min_training_cells) + 1;
            Unit::Bin adj_sz        = force_odd(sz);
            
            return std::max(adj_sz, min_window_sz);
        };
    };


    template <typename T>
    Window<T>::Window(
        const T*    min_bin, 
        const T*    max_bin, 
        Unit::Bin   window_size,
        Unit::Bin   guard_cells,
        Unit::dB    threshold_delta
    ) :
        delta       { threshold_delta },
        window_sz   { resize_window(window_size) },
        guard_sz    { guard_cells },
        range_start { min_bin },
        range_end   { max_bin }
    {
    }


    template <typename T>
    std::size_t Window<T>::size() const
    {
        return window_sz;
    }


    template <typename T>
    std::pair<Unit::dB, std::size_t> Window<T>::process_cell(const T* const elem_ptr)
    {
        //                               elem_ptr
        //             guard_sz <----->  v
        // _____________________________________________________________
        // |   |   |   |   |   | X | X |   | X | X |   |   |   |   |   |
        // -------------------------------------------------------------
        //   ^                   ^                   ^                    ^
        // lower_begin        lower_end         upper_begin       upper_end   
        //
        // At the beginning and end of the range the window must be 'slewed'
        // to ensure that all elements (elem_ptr) are checked.
        // If the window extends below the start of the range, truncate the
        // lower set of training cells and extend the upper set.  That is, 
        // the training cell set always remains the same size. Similarly,
        // if the upper training cells extend beyond the end of the range,
        // truncate the upper cells and extend the lower set to maintain the
        // window size.
        // Another way to think about this: Start with the cell-under-test (CUT)
        // at the left-most part of the window.  Increment the CUT until the
        // middle of the window is reached; after which the entire window can slide
        // up the azimuth. When the window reaches the end of the azimuth, continue
        // moving the CUT until the last cell in the window is tested.                    
        
        using namespace std;
        using namespace Unit;

        const T* lower_begin { elem_ptr - (window_sz / 2) };
        const T* lower_end   { elem_ptr - guard_sz };

        const T* upper_begin { elem_ptr + guard_sz + 1 };
        const T* upper_end   { elem_ptr + (window_sz / 2) + 1 };

        if (lower_begin < range_start) {
            lower_begin = range_start;
            upper_end   = lower_begin + window_sz;
        }

        if (upper_end > range_end) {
            upper_end = range_end;
            lower_begin = upper_end - window_sz;
        }

        if (lower_end < range_start) {
            lower_end = range_start;
        }

        if (upper_begin > upper_end) {
            upper_begin = range_end;
        }
        
        Sum_Ty      lower_sum    { accumulate(lower_begin, lower_end, T { }) };
        Sum_Ty      upper_sum    { accumulate(upper_begin, upper_end, T { }) };
        ptrdiff_t   elems        { (lower_end - lower_begin) + (upper_end - upper_begin) };
        Unit::dB    average      { static_cast<Unit::dB>(lower_sum + upper_sum) / elems };

        if (Traits::to_dB(*elem_ptr) > (average + delta)) {
            ++threshold_exceeded_count;
            return std::make_pair(Traits::to_dB(*elem_ptr), threshold_exceeded_count);
        }
        else {
            return std::make_pair(0, threshold_exceeded_count);
        }
    }


    // -----------------------------------------------------------------------------------------------------------------
    //
    template <typename T>
    class Cell_average {
    public:
        using Range_fn = std::function<Unit::Metre(Unit::Bin)>;

        Cell_average() = default;

        Cell_average(
            Unit::Bin       start_bin,
            Unit::Bin       window_size,
            Unit::Bin       guard_cells,
            Unit::dB        threshold_delta,
            const Range_fn& range_conv_fn    
        );

        Cell_average(
            Unit::Bin   start_bin,
            Unit::Bin   window_size,
            Unit::Bin   guard_cells,
            Unit::dB    threshold_delta 
        );

        std::vector<Point> points(const T* data, std::size_t sz) const;
        std::vector<Point> points(const std::vector<T>& vec) const;
        std::vector<Point> first_n_points(const T* data, std::size_t sz, std::size_t max_points) const;
        std::vector<Point> first_n_points(const std::vector<T>& vec, std::size_t max_points) const;

        std::vector<Unit::dB> process(const std::vector<T>& vec) const;
        std::vector<Unit::dB> process(const T* data, std::size_t sz) const;

    protected:
        Unit::Bin     start_bin     { 0 };
        Unit::Bin     window_sz     { 5 };
        Unit::Bin     guard_size    { 2 };
        Unit::dB      delta         { 0.0 };
        Range_fn      to_metre      { [](Unit::Bin b) { return b; } };
    };


    template <typename T>
    Cell_average<T>::Cell_average(
        Unit::Bin                       min_bin,
        Unit::Bin                       window_size,
        Unit::Bin                       guard_cells,
        Unit::dB                        threshold_delta,
        const Cell_average::Range_fn&   range_conv_fn
    ) :
        start_bin   { min_bin },
        window_sz   { window_size },
        guard_size  { guard_cells },
        delta       { threshold_delta },
        to_metre    { range_conv_fn }
    {
    }


    template <typename T>
    Cell_average<T>::Cell_average(
        Unit::Bin min_bin,
        Unit::Bin window_size,
        Unit::Bin guard_cells,
        Unit::dB  threshold_delta
    ) :
        start_bin   { min_bin },
        window_sz   { window_size },
        guard_size  { guard_cells },
        delta       { threshold_delta }
    {
    }


    template <typename T>
    std::vector<Point> Cell_average<T>::points(const std::vector<T>& vec) const
    {
        return first_n_points(vec.data(), vec.size(), vec.size());
    }


    template <typename T>
    std::vector<Point> Cell_average<T>::points(const T* fft_data, std::size_t sz) const
    {
        return first_n_points(fft_data, sz, sz);
    }

    
    template <typename T>
    std::vector<Point> Cell_average<T>::first_n_points(const std::vector<T>& vec, std::size_t max_points) const
    {
        return first_n_points(vec.data(), vec.size(), max_points);
    }


    template <typename T>
    std::vector<Point> Cell_average<T>::first_n_points(const T* fft_data, std::size_t sz, std::size_t max_points) const
    {
        using namespace Unit;

        std::vector<Point> output { };
        output.reserve(max_points);

        const T* start { fft_data + start_bin };
        const T* end   { fft_data + sz };

        Window<T> window {
            start,
            end,
            window_sz,
            guard_size,
            delta 
        };

        for (Bin bin { start_bin }; bin < sz; ++bin) {
            auto [power, count] = window.process_cell(fft_data + bin);
            
            if (power > 0_dB)        output.emplace_back(to_metre(bin), power);
            if (count == max_points) break;
        }

        return output;
    }


    template <typename T>
    std::vector<Unit::dB> Cell_average<T>::process(const std::vector<T>& vec) const
    {
        return process(vec.data(), vec.size());
    }


    template <typename T>
    std::vector<Unit::dB> Cell_average<T>::process(const T* fft_data, std::size_t sz) const
    {
        using namespace Unit;

        std::vector<dB> output { };
        output.resize(sz);

        const T* start { fft_data + start_bin };
        const T* end   { fft_data + sz };

        Window<T> window {
            start,
            end,
            window_sz,
            guard_size,
            delta 
        };

        for (Bin bin { start_bin }; bin < sz; ++bin) {
            auto [power, _] = window.process_cell(fft_data + bin);
            output[bin] = power;
        }

        return output;
    }

} // namespace Navtech::CFAR


#endif // CFAR_ALGORITHMS_H