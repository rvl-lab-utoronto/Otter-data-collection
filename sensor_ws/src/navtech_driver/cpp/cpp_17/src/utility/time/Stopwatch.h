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
#ifndef STOPWATCH_H
#define STOPWATCH_H

#include <cstddef>
#include <string_view>

#include "pointer_types.h"
#include "Statistical_value.h"
#include "Time_utils.h"
#include "Log.h"

namespace Navtech::Utility {

    // -----------------------------------------------------------------------------------------------------------------
    //
    template <std::size_t sz>
    class Stopwatch {
    public:
        Stopwatch() = default;
        Stopwatch(std::string_view name_str);
        void start();
        void stop();

    private:
        std::string name { "Stopwatch" };

        Statistical_value<Time::Tick_type, sz> times { };
        Time::Monotonic::Observation t_start { };
        Time::Monotonic::Observation t_end   { };

        int count { };
        void output();
    };


    template <std::size_t sz>
    Stopwatch<sz>::Stopwatch(std::string_view name_str) :
        name    { name_str }
    {
    }


    template <std::size_t sz>
    void Stopwatch<sz>::start()
    {
        t_start = Time::Monotonic::now();
    }


    template <std::size_t sz>
    void Stopwatch<sz>::stop()
    {
        t_end = Time::Monotonic::now();
        times.update((t_end - t_start).ticks());
        output();
    }


    template <std::size_t sz>
    void Stopwatch<sz>::output()
    {
        using Navtech::Utility::stdout_log;
        using Navtech::Utility::endl;
        
        if (++count == sz) {
            auto min        = Time::to_usec_duration(times.min());
            auto max        = Time::to_usec_duration(times.max());
            auto average    = Time::to_usec_duration(times.mean());

            stdout_log  << name << " - "
                        << "min [" << min.to_string() << "] "
                        << "max [" << max.to_string() << "] "
                        << "avg [" << average.to_string() << "] "
                        << endl;
            times.reset();
            count = 0;
        }
    }


    // -----------------------------------------------------------------------------------------------------------------
    //
    template <std::size_t sz>
    class Interval_timer {
    public:
        Interval_timer() = default;
        Interval_timer(std::string_view name_str);
        void update();

    private:
        std::string name { "Interval_timer" };

        Statistical_value<Time::Tick_type, sz> times { };
        Time::Monotonic::Observation t_curr { };
        Time::Monotonic::Observation t_prev { };

        int count { };
        void output();
    };


    template <std::size_t sz>
    Interval_timer<sz>::Interval_timer(std::string_view name_str) :
        name { name_str }
    {
    }


    template <std::size_t sz>
    void Interval_timer<sz>::update()
    {
        t_curr = Time::Monotonic::now();
        times.update((t_curr - t_prev).ticks());
        t_prev = t_curr;

        output();
    }


    template <std::size_t sz>
    void Interval_timer<sz>::output()
    {
        using Navtech::Utility::stdout_log;
        using Navtech::Utility::endl;
        
        if (++count == sz) {
            auto min        = Time::to_usec_duration(times.min());
            auto max        = Time::to_usec_duration(times.max());
            auto average    = Time::to_usec_duration(times.mean());

            stdout_log  << name << " - "
                        << "min [" << min.to_string() << "] "
                        << "max [" << max.to_string() << "] "
                        << "avg [" << average.to_string() << "] "
                        << endl;
            times.reset();
            count = 0;
        }
    }


    // -----------------------------------------------------------------------------------------------------------------
    //
    namespace Null {

        template <std::size_t sz>
        class Stopwatch {
        public:
            Stopwatch()                      { /* Do nothing */ }
            Stopwatch(std::string_view)      { /* Do nothing */ }
            void start()    { }
            void stop()     { }

        private:
        };


        template <std::size_t sz>
        class Interval_timer {
        public:
            Interval_timer()                    { /* Do nothing */ }
            Interval_timer(std::string_view)    { /* Do nothing */ }
            void update() { }
        private:
        };

    } // namespace Null

} // namespace Navtech::Utility

#endif // STOPWATCH_H