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
#ifndef COLOSSUS_UDP_MESSAGES_H
#define COLOSSUS_UDP_MESSAGES_H

#include "Colossus_UDP_message_base.h"
#include "net_conversion.h"
#include "IP_address.h"
#include "MAC_address.h"
#include "Units.h"
#include "Polar_coordinate.h"
#include "Time_utils.h"

using Navtech::Networking::to_uint16_host;
using Navtech::Networking::to_uint16_network;
using Navtech::Networking::to_uint32_host;
using Navtech::Networking::to_uint16_network;
using namespace Navtech::Unit;

namespace Navtech::Networking::Colossus_protocol::UDP {

// DO NOT REMOVE - 
// This ensures correct alignment for all 
// Colossus messages
//
#pragma pack(1)

    // ---------------------------------------------------------------------------------------------
    // Simple (no header or payload) messages
    //
   
   
    // ---------------------------------------------------------------------------------------------
    // Header and payload messages
    //
    class Discovery : public Message_base::Header_and_payload<Discovery> {
    public:
        void azimuth_samples(std::uint16_t val)     { azi_samples = to_uint16_network(val); }
        std::uint16_t azimuth_samples() const       { return to_uint16_host(azi_samples); }

        void bin_size(float val)                    { bin_sz = to_uint16_network(static_cast<std::uint16_t>(val * 10'000.0f)); }
        float bin_size() const                      { return (to_uint16_host(bin_sz) / 10'000.0f); }

        void range_in_bins(std::uint16_t val)       { range = to_uint16_network(val); }
        std::uint16_t range_in_bins() const         { return to_uint16_host(range); }

        void encoder_size(std::uint16_t val)        { encoder_sz = to_uint16_network(val); }
        std::uint16_t encoder_size() const          { return to_uint16_host(encoder_sz); }

        void ip_address(const IP_address& val)      { ip_addr = val.to_network_endian(); }
        IP_address ip_address() const               { return IP_address { ip_addr, Endian::network }; }

        void port(std::uint16_t val)                { tcp_port = to_uint16_network(val); }
        std::uint16_t port() const                  { return to_uint16_host(tcp_port); }

        void radar_serial(std::uint16_t val)        { radar_num = to_uint16_network(val); }
        std::uint16_t radar_serial() const          { return to_uint16_host(radar_num); }

        void mac_address(const MAC_address& val)    { mac_addr = val.to_byte_array(); }
        MAC_address mac_address() const             { return MAC_address { mac_addr }; }

        // All message classes *must* implement this function to
        // return the size of message header in bytes.
        // DO NOT return sizeof(Discovery)
        //
        std::size_t size() const        { return (sizeof(std::uint16_t) * 6) + (sizeof(std::uint32_t)) + (sizeof(std::uint8_t) * 6); }

    protected:
        std::uint16_t azi_samples;
		std::uint16_t bin_sz;
		std::uint16_t range;
		std::uint16_t encoder_sz;
		std::uint32_t ip_addr;
		std::uint16_t tcp_port;
		std::uint16_t radar_num;
		std::array<std::uint8_t, 6> mac_addr;
    };

    
    // ---------------------------------------------------------------------------------------------
    // Header-only messages
    //
    class Network_settings : public Message_base::Header_only<Network_settings> {
    public:
        void ip_address(const IP_address& val)      { ip_addr = val.to_network_endian(); }
        IP_address ip_address() const               { return IP_address { ip_addr, Endian::network }; }

        void subnet_mask(const IP_address& val)     { subnet = val.to_network_endian(); }
        IP_address subnet_mask() const              { return IP_address { subnet, Endian::network }; }

        void gateway(const IP_address& val)         { gateway_addr = val.to_network_endian(); }
        IP_address gateway() const                  { return IP_address { gateway_addr, Endian::network }; }

        void primary_DNS(const IP_address& val)     { primary_dns_addr = val.to_network_endian(); }
        IP_address primary_DNS() const              { return IP_address { primary_dns_addr, Endian::network }; }

        void secondary_DNS(const IP_address& val)   { secondary_dns_addr = val.to_network_endian(); }
        IP_address secondary_DNS() const            { return IP_address { secondary_dns_addr, Endian::network }; }

        void NTP_server(const IP_address& val)      { ntp_server_addr = val.to_network_endian(); }
        IP_address NTP_server() const               { return IP_address { ntp_server_addr, Endian::network }; }

        void syslog_server(const IP_address& val)   { syslog_server_addr = val.to_network_endian(); }
        IP_address syslog_server() const            { return IP_address { syslog_server_addr, Endian::network }; }

        std::size_t size() const                    { return (sizeof(std::uint32_t) * 7); }

    protected:
        // NOTE:
        // std::uint32_t => std::uint8_t[4], in network order
        //
        std::uint32_t ip_addr;
		std::uint32_t subnet;
		std::uint32_t gateway_addr;
		std::uint32_t primary_dns_addr;
		std::uint32_t secondary_dns_addr;
		std::uint32_t ntp_server_addr;
		std::uint32_t syslog_server_addr;
    };


    class Pointcloud_spoke : public Message_base::Header_and_payload<Pointcloud_spoke> {
    public:
        class Point {
        public:
            Point() = default;
            Point(Unit::Metre rng, Unit::dB pwr)
            {
                range(rng);
                power(pwr);
            }

            Unit::Metre range() const           { return static_cast<Unit::Metre>(to_float_host(range_val)); }
            void range(Unit::Metre val)         { range_val = to_uint32_network(static_cast<float>(val)); }

            Unit::dB power() const              { return (static_cast<Unit::dB>(to_float_host(power_val) / 2.0f)); }
            void power(Unit::dB val)            { power_val = to_uint32_network(static_cast<float>(val) * 2.0f); }

            std::uint8_t*       begin()   		{ return reinterpret_cast<std::uint8_t*>(this); }
            const std::uint8_t* begin() const	{ return reinterpret_cast<const std::uint8_t*>(this); }
            std::uint8_t*       end()		    { return (reinterpret_cast<std::uint8_t*>(this) + sizeof(Point)); }
            const std::uint8_t* end() const     { return (reinterpret_cast<const std::uint8_t*>(this) + sizeof(Point)); }

            static Point*       overlay_at(std::uint8_t* addr)       { return reinterpret_cast<Point*>(addr); }
            static const Point* overlay_at(const std::uint8_t* addr) { return reinterpret_cast<const Point*>(addr); }         
        
        private:
            std::uint32_t range_val;
            std::uint32_t power_val;  
        };


        std::uint16_t azimuth() const           { return to_uint16_host(azi); }
        void azimuth(std::uint16_t val)         { azi = to_uint16_network(val); }

        std::uint32_t seconds() const           { return to_uint32_host(ntp_sec); }
        void seconds(std::uint32_t val)         { ntp_sec = to_uint32_network(val); }

        std::uint32_t split_seconds() const     { return to_uint32_host(ntp_split_sec); }
        void split_seconds(std::uint32_t val)   { ntp_split_sec = to_uint32_network(val); }

        Unit::Degrees bearing() const           { return Unit::Degrees { to_float_host(bearing_val) }; }
        void bearing(const Unit::Degrees& val)  { bearing_val = to_uint32_network(val.to_float()); }


        Time::Real_time::Observation timestamp() const          
        { 
            return Time::Real_time::Observation { 
                timespec { 
                    static_cast<time_t>(to_uint32_host(ntp_sec)), 
                    static_cast<long>(to_uint32_host(ntp_split_sec)) 
                } 
            }; 
        }
        
        void timestamp(const Time::Real_time::Observation& t)   
        { 
            auto ntp      = t.to_ntp();
            ntp_sec       = to_uint32_network(static_cast<std::uint32_t>(ntp.tv_sec));
            ntp_split_sec = to_uint32_network(static_cast<std::uint32_t>(ntp.tv_nsec));
        }

        void points(std::size_t sz) {   point_count = static_cast<std::uint8_t>(sz); }
        
        std::pair<std::size_t, const Point*> points() const
        {
            if (point_count == 0) return { };

            return std::pair<std::size_t, const Point*> { 
                point_count,
                Point::overlay_at(&point_count + sizeof(point_count))
            };
        }
        
        // If your message has a header you MUST provide this function
        //
        std::size_t size() const { return (sizeof(std::uint8_t) * 1) + (sizeof(std::uint16_t) * 1) + (sizeof(std::uint32_t) * 3); }

    private:
        // Attribute order MUST match the actual message header, as
        // this is a memory overlay.
        //
        std::uint16_t azi;
        std::uint32_t ntp_sec;
        std::uint32_t ntp_split_sec;
        std::uint32_t bearing_val;
        std::uint8_t  point_count;
    };

    inline std::vector<std::uint8_t>& operator<<(std::vector<std::uint8_t>& v, Pointcloud_spoke::Point& pt)
    {
        v.insert(v.end(), pt.begin(), pt.end());
        return v;
    }


    inline std::vector<std::uint8_t>& operator<<(std::vector<std::uint8_t>& v, Pointcloud_spoke::Point&& pt)
    {
        v.insert(v.end(), pt.begin(), pt.end());
        return v;
    }
   

    // ---------------------------------------------------------------------------------------------
    // Payload-only messages
    //


// DO NOT REMOVE - 
// This ensures correct alignment for all 
// Colossus messages
//
#pragma pack()

} // namespace Navtech::Networking::Colossus_protocol::UDP


#endif // COLOSSUS_UDP_MESSAGES_H