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
#ifndef NAVIGATION_AREA_RULE_H
#define NAVIGATION_AREA_RULE_H

#include <vector>
#include <cstdint>

#include "net_conversion.h"

namespace Navtech::Navigation {
#pragma pack(1)
		
    class Header {
    public:
        uint8_t rule_count() const		    { return count; }
        void    rule_count(std::uint8_t val){ count = val; }

        bool    enable_health() const       { return health_enabled; }
        void    enable_health(bool val)     { health_enabled = (val ? 1 : 0); }

        bool    failsafe_mode() const       { return failsafe; }
        void    failsafe_mode(bool val)     { failsafe = (val ? 1 : 0); }

        std::uint8_t*       begin()	  	    { return reinterpret_cast<std::uint8_t*>(this); }
        const std::uint8_t* begin() const   { return reinterpret_cast<const std::uint8_t*>(this); }
        std::uint8_t*       end() 		    { return (reinterpret_cast<std::uint8_t*>(this) + sizeof(Header)); }
        const std::uint8_t* end() const     { return (reinterpret_cast<const std::uint8_t*>(this) + sizeof(Header)); }

        static Header*       overlay_at(std::uint8_t* const addr) { return reinterpret_cast<Header*>(addr); }
        static const Header* overlay_at(const std::uint8_t* const addr) { return reinterpret_cast<const Header*>(addr); }
    
    private:
        std::uint8_t count { };
        std::uint8_t health_enabled { };
        std::uint8_t failsafe       { };
    };


    inline std::vector<std::uint8_t>& operator<<(std::vector<std::uint8_t>& v, Header& hdr)
    {
        v.insert(v.end(), hdr.begin(), hdr.end());
        return v;
    }



    class Point {
    public:
        Point() = default;

        Point(float x_init, float y_init)
        {
            x(x_init);
            y(y_init);
        }

        void x(float val)
        { 
            x_val = Networking::to_uint16_network(static_cast<std::uint16_t>(val * 10.0f));
        }
        
        float x() const 
        {
            std::int16_t val = static_cast<std::int16_t>(Networking::to_uint16_host(x_val));
            return (val / 10.0f);
        }

        void y(float val)
        {
            y_val = Networking::to_uint16_network(static_cast<std::uint16_t>(val * 10.0f));
        }

        float y() const
        {
            std::int16_t val = static_cast<std::int16_t>(Networking::to_uint16_host(y_val));
            return (val / 10.0f);
        }

        bool operator==(const Point& rhs) const
        {
            return ((this->x() == rhs.x()) && (this->y() == rhs.y()));
        }

        bool operator!=(const Point& rhs) const
        {
            return (!(*this == rhs));
        }

        std::uint8_t*       begin()   		{ return reinterpret_cast<std::uint8_t*>(this); }
        const std::uint8_t* begin() const	{ return reinterpret_cast<const std::uint8_t*>(this); }
        std::uint8_t*       end()		    { return (reinterpret_cast<std::uint8_t*>(this) + sizeof(Point)); }
        const std::uint8_t* end() const     { return (reinterpret_cast<const std::uint8_t*>(this) + sizeof(Point)); }

        static Point*       overlay_at(std::uint8_t* addr)       { return reinterpret_cast<Point*>(addr); }
        static const Point* overlay_at(const std::uint8_t* addr) { return reinterpret_cast<const Point*>(addr); }

    private:
        std::uint16_t x_val { };
        std::uint16_t y_val { };
    };


    inline std::vector<std::uint8_t>& operator<<(std::vector<std::uint8_t>& v, Point& pt)
    {
        v.insert(v.end(), pt.begin(), pt.end());
        return v;
    }


    inline std::vector<std::uint8_t>& operator<<(std::vector<std::uint8_t>& v, Point&& pt)
    {
        v.insert(v.end(), pt.begin(), pt.end());
        return v;
    }


    class Rule {
    public:
        std::uint8_t id() const
        {
            return id_val;
        }

        void id(std::uint8_t val)
        {
            id_val = val;
        }

        bool enabled() const
        {
            return (is_enabled != 0);
        }

        void enabled(bool val)
        {
            is_enabled = val;
        }

        bool invert_break_logic() const
        {
            return (is_invert_break_logic != 0);
        }

        void invert_break_logic(bool val)
        {
            is_invert_break_logic = val;
        }

        float threshold_delta() const
        {
            return static_cast<int16_t>(Networking::to_uint16_host(threshold_delta_val)) / 10.0f;
        }

        void threshold_delta(float val)
        {
            threshold_delta_val = Networking::to_uint16_network(static_cast<std::uint16_t>(val * 10.0f));
        }

        std::uint16_t break_allowance() const
        {
            return Networking::to_uint16_host(break_allowance_val);
        }

        void break_allowance(std::uint16_t val)
        {
            break_allowance_val = Networking::to_uint16_network(val);
        }

        std::uint16_t allowance_curve_decrement() const
        {
            return Networking::to_uint16_host(allowance_curve_decr_val);
        }

        void allowance_curve_decrement(std::uint16_t val)
        {
            allowance_curve_decr_val = Networking::to_uint16_network(val);
        }
        
        std::size_t point_count() const
        {
            return Networking::to_uint16_host(point_count_val);
        }

        void point_count(std::size_t pts)
        {
            point_count_val = Networking::to_uint16_network(static_cast<std::uint16_t>(pts));

            // Update the overall record length as well
            //
            length = Networking::to_uint32_network(header_size() + static_cast<std::uint32_t>(pts * sizeof(Point)));
        }
        
        std::pair<const Point*, std::size_t> points() const
        {
            return std::make_pair(
                Point::overlay_at(header_end()),
                point_count()
            );
        }

        std::uint8_t*       begin()					{ return reinterpret_cast<std::uint8_t*>(this); }
        const std::uint8_t* begin() const			{ return reinterpret_cast<const std::uint8_t*>(this); }

        std::uint8_t* 		end()		    		{ return (reinterpret_cast<std::uint8_t*>(this) + sizeof(length) + Networking::to_uint32_host(length)); }
        const std::uint8_t* end() const	    		{ return (reinterpret_cast<const std::uint8_t*>(this) + sizeof(length) + Networking::to_uint32_host(length)); }

        std::uint8_t* 		header_begin()			{ return begin(); }
        const std::uint8_t* header_begin()	const 	{ return begin(); }
        std::uint8_t*  		header_end()		    { return (reinterpret_cast<std::uint8_t*>(this) + sizeof(length) + header_size()); }
        const std::uint8_t*	header_end() const	    { return (reinterpret_cast<const std::uint8_t*>(this) + sizeof(length) + header_size()); }

        std::uint8_t* 		points_begin()			{ return header_end();	}
        const std::uint8_t*	points_begin() const	{ return header_end();	}
        std::uint8_t* 		points_end()			{ return end(); }
        const std::uint8_t*	points_end() const		{ return end(); }

        static Rule* overlay_at(std::uint8_t* addr) 			{ return reinterpret_cast<Rule*>(addr); }
        static const Rule* overlay_at(const std::uint8_t* addr)	{ return reinterpret_cast<const Rule*>(addr); }

    private:
        std::uint32_t length                    { Networking::to_uint32_network(header_size()) };
        std::uint8_t  id_val                    { };
        std::uint8_t  is_enabled                { };
        std::uint8_t  is_invert_break_logic     { };
        std::uint16_t threshold_delta_val       { };
        std::uint16_t break_allowance_val       { };
        std::uint16_t allowance_curve_decr_val  { };
        std::uint16_t point_count_val           { };

        constexpr std::uint32_t header_size() const
        {
            return static_cast<std::uint32_t>(sizeof(Rule) - sizeof(std::uint32_t));
        }
    };


    inline std::vector<std::uint8_t>& operator<<(std::vector<std::uint8_t>& v, Rule& r)
    {
        v.insert(v.end(), r.header_begin(), r.header_end());
        return v;
    }

#pragma pack()
} // namespace Navtech::Messaging


#endif // NAVIGATION_AREA_RULE_H