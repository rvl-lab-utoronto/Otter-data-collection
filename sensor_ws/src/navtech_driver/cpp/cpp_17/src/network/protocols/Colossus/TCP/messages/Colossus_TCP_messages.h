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
#ifndef COLOSSUS_TCP_MESSAGES_H
#define COLOSSUS_TCP_MESSAGES_H

// DEBUG ONLY
//
#include <iomanip>
#include <iostream>
#include <cstdint>
#include "Log.h"

#include "Colossus_TCP_message_base.h"
#include "net_conversion.h"
#include "Time_utils.h"
#include "Units.h"

using Navtech::Networking::to_uint16_host;
using Navtech::Networking::to_uint16_network;
using Navtech::Networking::to_uint32_host;
using Navtech::Networking::to_uint16_network;

namespace Navtech::Networking::Colossus_protocol::TCP {

// DO NOT REMOVE - 
// This ensures correct alignment for all 
// Colossus messages
//
#pragma pack(1)

    // ---------------------------------------------------------------------------------------------
    // Simple (no header or payload) messages
    //
    class Start_radar : public Message_base::Simple<Start_radar> { };
    class Stop_radar  : public Message_base::Simple<Stop_radar>  { };


    // ---------------------------------------------------------------------------------------------
    // Header and payload messages
    //

    class Configuration : public Message_base::Header_and_payload<Configuration> {
    public:
        // Accessor/mutator API; or, you could make the attributes public
        // (but be careful of endianness issues!)
        //
        std::uint16_t azimuth_samples() const       { return to_uint16_host(azi_samples); }
        void azimuth_samples(std::uint16_t val)     { azi_samples = to_uint16_network(val); }

        std::uint16_t bin_size() const              { return to_uint16_host(bin_sz); }
        void bin_size(std::uint16_t val)            { bin_sz = to_uint16_network(val); }

        std::uint16_t range_in_bins() const         { return to_uint16_host(range_bins); }
        void range_in_bins(std::uint16_t val)       { range_bins = to_uint16_network(val); }

        std::uint16_t encoder_size() const          { return to_uint16_host(encoder_sz); }
        void encoder_size(std::uint16_t val)        { encoder_sz = to_uint16_network(val); }

        std::uint16_t rotation_speed() const        { return to_uint16_host(rotation_spd); }
        void rotation_speed(std::uint16_t val)      { rotation_spd = to_uint16_network(val); }

        std::uint16_t packet_rate() const           { return to_uint16_host(pckt_rate); }
        void packet_rate(std::uint16_t val)         { pckt_rate = to_uint16_network(val); }

        float range_gain() const                    { return to_float_host(gain); }
        void range_gain(float val)                  { gain = to_uint32_network(val); }

        float range_offset() const                  { return to_float_host(offset); }
        void range_offset(float val)                { offset = to_uint32_network(val); }

        // If your message has a header you MUST provide this function
        //
        std::size_t size() const { return (6 * sizeof(std::uint16_t) + 2 * sizeof(std::uint32_t)); }

    private:
        // Attribute order MUST match the actual message header, as
        // this is a memory overlay.
        //
        std::uint16_t azi_samples;
        std::uint16_t bin_sz;
        std::uint16_t range_bins;
        std::uint16_t encoder_sz;
        std::uint16_t rotation_spd;
        std::uint16_t pckt_rate;
        std::uint32_t gain;
        std::uint32_t offset;
    };


    class FFT_data : public Message_base::Header_and_payload<FFT_data> {
    public:
        // Accessor/mutator API; or, you could make the attributes public
        // (but be careful of endianness issues!)
        //
        std::uint16_t fft_data_offset() const                   { return to_uint16_host(data_offset); }
        void fft_data_offset(std::uint16_t val)                 { data_offset = to_uint16_network(val); }

        std::uint16_t sweep_counter() const                     { return to_uint16_host(sweep); }
        void sweep_counter(std::uint16_t val)                   { sweep = to_uint16_network(val); }

        std::uint16_t azimuth() const                           { return to_uint16_host(azi); }
        void azimuth(std::uint16_t val)                         { azi = to_uint16_network(val); }

        std::uint32_t ntp_seconds() const                       { return seconds; }
        void ntp_seconds(std::uint32_t val)                     { seconds = val; }

        std::uint32_t ntp_split_seconds() const                 { return split_seconds; }
        void ntp_split_seconds(std::uint32_t val)               { split_seconds = val; }

        Time::Real_time::Observation timestamp() const          {
                                                                    timeval t_val { 
                                                                        static_cast<long>(ntp_seconds()),
                                                                        static_cast<long>(ntp_split_seconds())
                                                                    };
                                                                    auto duration = Time::Duration      { t_val };
                                                                    return Time::Real_time::Observation { duration };
                                                                }
        
        void timestamp(const Time::Real_time::Observation& t)   {
                                                                    auto ntp = t.to_ntp();
                                                                    ntp_seconds(static_cast<uint32_t>(ntp.tv_sec));
                                                                    ntp_split_seconds(ntp.tv_nsec);
                                                                }

        // If your message has a header you MUST provide this function
        //
        constexpr std::size_t size() const { return (3 * sizeof(std::uint16_t) + 2 * sizeof(std::uint32_t)); }

    private:
        // Attribute order MUST match the actual message header, as
        // this is a memory overlay.
        //
        std::uint16_t data_offset { to_uint16_network(static_cast<std::uint16_t>(size())) };
        std::uint16_t sweep;
        std::uint16_t azi;
        std::uint32_t seconds;
        std::uint32_t split_seconds;
    };


    class Navigation_data : public Message_base::Header_and_payload<Navigation_data> {
    public:
        class Point {
        public:
            Unit::Metre range() const           { return static_cast<Unit::Metre>(to_uint32_host(range_val) / 1e6f); }
            void        range(Unit::Metre val)  { range_val = to_uint32_network(val * 1'000'000); }

            Unit::dB power() const              { return (static_cast<Unit::dB>(to_uint16_host(power_val) / 10.0f)); }
            void power(Unit::dB val)            { power_val = to_uint16_network(static_cast<uint16_t>(val * 10.0f)); }

            std::uint8_t*       begin()   		{ return reinterpret_cast<std::uint8_t*>(this); }
            const std::uint8_t* begin() const	{ return reinterpret_cast<const std::uint8_t*>(this); }
            std::uint8_t*       end()		    { return (reinterpret_cast<std::uint8_t*>(this) + sizeof(Point)); }
            const std::uint8_t* end() const     { return (reinterpret_cast<const std::uint8_t*>(this) + sizeof(Point)); }

            static Point*       overlay_at(std::uint8_t* addr)       { return reinterpret_cast<Point*>(addr); }
            static const Point* overlay_at(const std::uint8_t* addr) { return reinterpret_cast<const Point*>(addr); }

            static std::size_t  size()  { return sizeof(std::uint32_t) + sizeof(std::uint16_t); }
        private:
            std::uint32_t           range_val;
            std::uint16_t           power_val;
        };

        // Accessor/mutator API; or, you could make the attributes public
        // (but be careful of endianness issues!)
        //
        std::uint16_t azimuth() const               { return to_uint16_host(net_azimuth); }
        void azimuth(std::uint16_t val)             { net_azimuth = to_uint16_network(val); }

        std::uint32_t ntp_seconds() const           { return to_uint32_host(seconds); }
        void ntp_seconds(std::uint32_t val)         { seconds = to_uint32_network(val); }

        std::uint32_t ntp_split_seconds() const     { return to_uint32_host(split_seconds); }
        void ntp_split_seconds(std::uint32_t val)   { split_seconds = to_uint32_network(val); }

        // If your message has a header you MUST provide this function
        //
        std::size_t size() const { return (sizeof(std::uint16_t) + 2 * sizeof(std::uint32_t)); }

        std::pair<std::size_t, const Point*> points() const {
            using Navtech::Utility::stdout_log;
            using Navtech::Utility::endl;
            auto point_count = std::distance(protobuf_begin(), protobuf_end()) / Point::size();

            if (point_count == 0) return { };

            return std::pair<std::size_t, const Point*> {
                point_count,
                Point::overlay_at(Header::end())
            };
        }
    private:
        // Attribute order MUST match the actual message header, as
        // this is a memory overlay.
        //
        std::uint16_t net_azimuth;
        std::uint32_t seconds;
        std::uint32_t split_seconds;
    };

    // ---------------------------------------------------------------------------------------------
    // Header-only messages
    //

    class Set_navigation_threshold : public Message_base::Header_only<Set_navigation_threshold> {
    public:
        std::uint16_t threshold() const             { return to_uint16_host(nav_threshold); }
        void threshold(std::uint16_t value)         { nav_threshold = to_uint16_network(value); }

        std::size_t size() const                    { return sizeof(std::uint16_t); }

    private:
        std::uint16_t nav_threshold { };
    };


    class Navigation_config : public Message_base::Header_only<Navigation_config> {
    public:
        std::size_t size() const
        {
            return (sizeof(operating_bins) + sizeof(min_bin) + sizeof(threshold) + sizeof(max_peaks));
        }

        void bins_to_operate_on(std::uint16_t bins)     { operating_bins = to_uint16_network(bins); }
        std::uint16_t bins_to_operate_on() const        { return to_uint16_host(operating_bins); }

        void min_bin_to_operate_on(std::uint16_t min)   { min_bin = to_uint16_network(min); }
        std::uint16_t min_bin_to_operate_on() const     { return to_uint16_host(min_bin); }

        void navigation_threshold(float level)          { threshold = to_uint32_network(level * 10.0F); }
        float navigation_threshold() const              { return (to_float_host(threshold) / 10.0F); }

        void max_peaks_per_azimuth(std::uint32_t peaks) { max_peaks = to_uint32_network(peaks); }
        std::uint32_t max_peaks_per_azimuth() const     { return to_uint32_host(max_peaks); }

    private:
        // Attribute overlay
        //
        std::uint16_t operating_bins;
        std::uint16_t min_bin;
        std::uint32_t threshold;
        std::uint32_t max_peaks;
    };


    class Set_autotune : public Message_base::Header_only<Set_autotune> {
    public:
        void autotune_value(std::uint16_t val)          { value = to_uint16_network(val); }
        std::uint16_t autotune_value() const            { return to_uint16_host(value); }

        std::size_t size() const { return sizeof(std::uint16_t); }

    private:
        std::uint16_t value;
    };


    class Accelerometer_data : public Message_base::Header_only<Accelerometer_data> {
    public:
        void  theta(float val)                          { theta_val = to_uint32_network(val); }
        float theta() const                             { return to_float_host(theta_val); }

        void  psi(float val)                            { psi_val = to_uint32_network(val); }
        float psi() const                               { return to_float_host(psi_val); }

        void  phi(float val)                            { phi_val = to_uint32_network(val); }
        float phi() const                               { return to_float_host(phi_val); }

        std::size_t size() const                        { return sizeof(std::uint32_t) * 3; }

    private:
        std::uint32_t theta_val;
        std::uint32_t psi_val;
        std::uint32_t phi_val;
    };


    class Set_navigation_gain_and_offset : public Message_base::Header_only<Set_navigation_gain_and_offset> {
    public:
        void  gain(float val)                   { gain_val = to_uint32_network(static_cast<uint32_t>(val * scale_factor)); }
        float gain() const                      { return static_cast<std::uint32_t>(to_uint32_host(gain_val)) / scale_factor; }

        void  offset(float val)                 { offset_val = to_uint32_network(static_cast<uint32_t>(val * scale_factor)); }
        float offset() const                    { return static_cast<std::int32_t>(to_uint32_host(offset_val)) / scale_factor; }

        std::size_t size() const                { return sizeof(std::uint32_t) * 2; }
    
    private:
        static constexpr float scale_factor { 1'000'000.0f };
        std::uint32_t gain_val;
        std::uint32_t offset_val;
    };


    namespace Core::Configuration::Radar {

        enum class Navigation_mode : std::uint8_t {
            buffer_off,
            buffer_average,
            buffer_max
        };
    }

    class Set_navigation_buffer_config : public Message_base::Header_only<Set_navigation_buffer_config> {
    public:
        void mode(Core::Configuration::Radar::Navigation_mode val)  { mode_val = static_cast<std::uint8_t>(val); }
        Core::Configuration::Radar::Navigation_mode mode() const    { return static_cast<Core::Configuration::Radar::Navigation_mode>(mode_val); }

        void length(std::uint32_t val)                              { length_val = to_uint32_network(val); }
        std::uint32_t length() const                                { return to_uint32_host(length_val); }

        std::size_t size() const                                    { return sizeof(std::uint8_t) + sizeof(std::uint32_t); }
    
    private:
        std::uint8_t  mode_val;
        std::uint32_t length_val;
    };


    class Set_nav_bin_operation : public Message_base::Header_only<Set_nav_bin_operation> {
    public:
        void bins_to_operate_on(std::uint8_t val)                   { bin_range = val; }
        std::uint8_t bins_to_operate_on() const                     { return bin_range; }

        void min_bin_to_operate_on(std::uint16_t val)               { min_bin = to_uint16_network(val); }
        std::uint16_t min_bin_to_operate_on() const                 { return to_uint16_host(min_bin); }

        std::size_t size() const                                    { return sizeof(std::uint8_t) + sizeof(std::uint16_t); }
    
    private:
        std::uint8_t  bin_range;
        std::uint16_t min_bin;
    };


    class Time_server_status : public Message_base::Header_only<Time_server_status> {
    public:
        void ntp_enabled(bool val)                      { is_ntp_enabled = val; }
        bool ntp_enabled() const                        { return is_ntp_enabled; }

        void ntp_sync(bool val)                         { is_ntp_syncd = val; }
        bool ntp_sync() const                           { return is_ntp_syncd; }

        void ntp_remote(IP_address ip_addr)             { ntp_remote_addr = ip_addr.to_network_endian(); }
        IP_address ntp_remote() const                   { return IP_address { ntp_remote_addr, Endian::network }; }

        void ptp_enabled(bool val)                      { is_ptp_enabled = val; }
        bool ptp_enabled() const                        { return is_ptp_enabled; }

        void ptp_sync(bool val)                         { is_ptp_syncd = val; }
        bool ptp_sync() const                           { return is_ptp_syncd; }

        void ptp_remote(IP_address ip_addr)             { ptp_remote_addr = ip_addr.to_network_endian(); }
        IP_address ptp_remote() const                   { return IP_address { ptp_remote_addr, Endian::network }; }

        void now(const Time::Real_time::Observation& t) { 
                                                            now_sec  = to_uint32_network(static_cast<std::uint32_t>(t.to_ntp().tv_sec));
                                                            now_nsec = to_uint32_network(static_cast<std::uint32_t>(t.to_ntp().tv_nsec));
                                                        }

        Time::Real_time::Observation now() const        {
                                                            timespec t { };
                                                            t.tv_sec  = to_uint32_host(now_sec);
                                                            t.tv_nsec = to_uint32_host(now_nsec);
                                                            return Time::Real_time::Observation { t };
                                                        }

        std::size_t size() const                        { return (sizeof(std::uint8_t) * 4) + (sizeof(std::uint32_t) * 4); }

    private:
        std::uint8_t  is_ntp_enabled;
        std::uint8_t  is_ntp_syncd;
        std::uint32_t ntp_remote_addr;
        std::uint8_t  is_ptp_enabled;
        std::uint8_t  is_ptp_syncd;
        std::uint32_t ptp_remote_addr;
        std::uint32_t now_sec;
        std::uint32_t now_nsec;
    };

    // ---------------------------------------------------------------------------------------------
    // Payload-only messages
    //

    class Health                : public Message_base::Payload_only<Health> { };
    class Contour_info          : public Message_base::Payload_only<Contour_info> { };
    class Sector_blanking       : public Message_base::Payload_only<Sector_blanking> { };
    class Navigation_alarm_data : public Message_base::Payload_only<Navigation_alarm_data> { };
    class Navigation_area_rules : public Message_base::Payload_only<Navigation_area_rules> { };

// DO NOT REMOVE - 
// This ensures correct alignment for all 
// Colossus messages
//
#pragma pack()

} // namespace Navtech::Networking::Colossus_protocol::TCP


#endif // COLOSSUS_TCP_MESSAGES_H
