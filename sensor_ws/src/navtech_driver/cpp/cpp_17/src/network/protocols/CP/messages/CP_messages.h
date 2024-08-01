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
#ifndef CP_MESSAGES_H
#define CP_MESSAGES_H

#include <cstdint>

#ifdef __linux__
#include <arpa/inet.h>
#elif _WIN32
#include <winsock2.h>
#endif

#include "CP_message_base.h"


namespace Navtech::Networking::CP_protocol {

    #pragma pack(1)

    // ---------------------------------------------------------------------------------------------
    // Actual message types must inherit from one of the Message_base templates
    // passing their own type as the template parameter.  This is an application
    // of the Curiously Recurring Template Pattern (CRTP)
    //
    class Test_message : public Message_base::Header_and_payload<Test_message> {
    public:
        // Accessor/mutator API; or, you could make the attributes public
        // (but be careful of endianness issues!)
        //
        std::uint16_t azimuth_samples() const   { return ntohs(azi_samples); } 
        void azimuth_samples(std::uint16_t val) { azi_samples = htons(val); }

        std::uint16_t bin_size() const          { return ntohs(bin_sz); }
        void bin_size(std::uint16_t val)        { bin_sz = htons(val); }

        std::uint16_t range_in_bins() const     { return ntohs(range_bins); }
        void range_in_bins(std::uint16_t val)   { range_bins = htons(val); }
        
        std::uint16_t encoder_size() const      { return ntohs(encoder_sz); }
        void encoder_size(std::uint16_t val)    { encoder_sz = htons(val); }

        std::uint16_t rotation_speed() const    { return ntohs(rotation_spd); }
        void rotation_speed(std::uint16_t val)  { rotation_spd = htons(val); }

        std::uint16_t packet_rate() const       { return ntohs(pckt_rate); }
        void packet_rate(std::uint16_t val)     { pckt_rate = htons(val); }

        float range_gain() const                { return gain; }
        void range_gain(float val)              { gain = val; }

        float range_offset() const              { return offset; } 
        void range_offset(float val)            { offset = val; }


        // If your message has a header you MUST provide this function
        //
        std::size_t size() const
        {
            return (
                6 * sizeof(std::uint16_t) +
                2 * sizeof(float) 
            );
        }

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
        float         gain;
        float         offset;    
    };


    class Set_navigation_threshold : public Message_base::Header_only<Set_navigation_threshold> {
    public:
        std::uint16_t threshold() const     
        {
            return ntohs(threshold_val); 
        }

        void threshold(std::uint16_t val)   
        { 
            threshold_val = htons(val); 
        }

        std::size_t size() const     
        { 
            return sizeof(std::uint16_t); 
        }
        
    private:
        std::uint16_t threshold_val;
    };
    
    //
    // ---------------------------------------------------------------------------------------------


    class Configuration :                                       public Message_base::Payload_only<Configuration> { };
    class Health :                                              public Message_base::Payload_only<Health> { };
    class Contour_update :                                      public Message_base::Payload_only<Contour_update> { };
    class Tracker_distribution_update :                         public Message_base::Payload_only<Tracker_distribution_update> { };
    class Tracker_playback_command :                            public Message_base::Payload_only<Tracker_playback_command> { };
    class Static_target_detection_capture_baseline :            public Message_base::Payload_only<Static_target_detection_capture_baseline> { };
    class Static_target_detection_capture_threshold_request :   public Message_base::Payload_only<Static_target_detection_capture_threshold_request> { };
    class CP_licence_request :                                  public Message_base::Payload_only<CP_licence_request> { };


    #pragma pack()

} // namespace Navtech::Networking::CP_protocol

#endif // CP_MESSAGES_H