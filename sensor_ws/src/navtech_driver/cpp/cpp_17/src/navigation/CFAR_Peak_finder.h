#ifndef CFAR_PEAK_FINDER_H
#define CFAR_PEAK_FINDER_H

#include <string_view>

#include "Colossus_protocol.h"
#include "Units.h"
#include "File_writer.h"
#include "CFAR_algorithms.h"
#include "configurationdata.pb.h"
// #include "Active.h"

namespace Navtech::Navigation {

    enum class Subresolution_mode { curve_fit, centre_of_mass, centre_of_mass_2d };

    struct CFAR_Target {
        CFAR_Target(float bearing, float range) :
            bearing(bearing), range(range) 
            {}
        
        float bearing;
        float range;
    };


    class CFAR_Peak_finder : public Navtech::Utility::Active {
    public:
        using float_it = std::vector<float>::iterator;

        void configure(
            const Networking::Colossus_protocol::TCP::Configuration&        cfg_msg,
            const Networking::Colossus_protocol::TCP::Navigation_config     nav_cfg,
            std::function<void(const CFAR_Target&)>                         callback,
            Subresolution_mode                                              subresolution_mode
        );
    
        void find_peaks(
            const Networking::Colossus_protocol::TCP::FFT_data&    fft_msg
        );
        
    private:
        float           range_gain                      { 0.0f };
        Unit::Metre     range_offset                    { 0.0f };
        Unit::Metre     range_resolution                { 0.1752f };
        Unit::Metre     min_range                       { 0.0 };
        Unit::Metre     max_range                       { 50.0 };
        
        static const    Unit::Bin   max_bins_to_operate_on   { 15 };

        Unit::Azimuth   steps_per_azimuth               { 5600 / 400 };
        Unit::Azimuth   azimuth_samples                 { 400 };
        Unit::Bin       range_in_bins                   { 2856 };

        float           azimuth_to_bearing              { 360.0f * 400.f/5600.f };
        Unit::Bin       minimum_bin                     { 0 };
        Unit::Bin       max_peaks                       { 5 };

        CFAR::Cell_average<float> cfar                  { };

        Subresolution_mode mode { Subresolution_mode::curve_fit };

        std::function<float(float)>                 to_degrees      { nullptr };
        std::function<Unit::Metre(float)>           to_metre        { nullptr };
        std::function<void(const CFAR_Target&)>     target_callback { nullptr };

        std::vector<std::vector<float>>     rotation_data   { };
        std::uint16_t                       last_azimuth    { 0 };
        std::uint32_t                       rotations       { 0 };

        void on_find_peaks(
            const Networking::Colossus_protocol::TCP::FFT_data&    fft_msg
        );

        void send_target(float resolved_bin, float resolved_bearing);
    
        float peak_resolve(
            const std::vector<float>& data,
            Unit::Bin peak_bin,
            Unit::Bin window_sz
        );

        void find_shapes(
            const std::vector<std::vector<float>> rotation_data
        );

   };

} // namespace Navtech::Navigation
#endif