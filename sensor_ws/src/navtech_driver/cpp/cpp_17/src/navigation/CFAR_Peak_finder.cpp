#include "CFAR_Peak_finder.h"
#include "Vector_maths.h"
#include "Log.h"
#include "float_equality.h"
#include "Centre_of_mass.h"
#include "Shape_finder.h"
namespace Navtech::Navigation {

    void CFAR_Peak_finder::configure(
        const Navtech::Networking::Colossus_protocol::TCP::Configuration&       cfg_msg,
        const Navtech::Networking::Colossus_protocol::TCP::Navigation_config    nav_cfg,
        std::function<void(const CFAR_Target&)>                                 callback,
        Subresolution_mode                                                      subresolution_mode
    ) 
    {
        using namespace Navtech::CFAR;
        // Set up the CFAR algorithm
        // 
        range_in_bins           = cfg_msg.range_in_bins();
        range_gain              = cfg_msg.range_gain();
        range_offset            = cfg_msg.range_offset();
        range_resolution        = cfg_msg.bin_size() / 10000.0f;

        minimum_bin             = nav_cfg.min_bin_to_operate_on();
        max_peaks               = nav_cfg.max_peaks_per_azimuth();
        min_range               = minimum_bin * range_resolution;
        max_range               = range_in_bins * range_resolution;
        azimuth_samples         = cfg_msg.azimuth_samples();
        steps_per_azimuth       = cfg_msg.encoder_size() / azimuth_samples;

        mode                    = subresolution_mode;
        
        if (mode == Subresolution_mode::centre_of_mass_2d) {
            rotation_data.resize(azimuth_samples);
        }

        to_degrees = [this] (float a)  { 
            return fmod(a * 360.0f / static_cast<float>(azimuth_samples) + 360.0f, 360.0f); 
        };
        
        to_metre        = [this] (float b)      { return (b * range_gain * range_resolution) + range_offset; };
        target_callback = callback;

        cfar =  Cell_average<float>(
                    nav_cfg.min_bin_to_operate_on(),
                    nav_cfg.bins_to_operate_on(),
                    std::uint16_t { 3 }, // num guard cells
                    nav_cfg.navigation_threshold(),
                    to_metre
        );

    }


    void CFAR_Peak_finder::find_peaks(
        const Networking::Colossus_protocol::TCP::FFT_data&    fft_msg
    )
    {
        async_call(
            &CFAR_Peak_finder::on_find_peaks,
            this,
            fft_msg
        );
    }


    void CFAR_Peak_finder::on_find_peaks(            
        const Networking::Colossus_protocol::TCP::FFT_data&    fft_msg
    )
    {
        using Utility::essentially_equal;
        using Utility::stdout_log;
        using Utility::endl;

        auto fft_data = fft_msg.to_vector();

        std::vector<float> power_data(fft_data.size());
                
       

        std::transform(fft_data.begin(), fft_data.end(), 
                        power_data.begin(), 
                        [] (std::uint8_t f) { return f * 0.5f; });
        
        auto cfar_points = cfar.process(power_data);
        // It's possible for the data to be contoured
        //
        cfar_points.resize(range_in_bins);
                
        auto min_it = cfar_points.begin() + minimum_bin;
        auto max_it = std::max_element(cfar_points.begin() + minimum_bin, cfar_points.end());

        // This is so rows aren't skipped, there's certainly a better way of doing this
        //
        if (Utility::essentially_equal(*max_it, 0.0f) && mode != Subresolution_mode::centre_of_mass_2d) return;

        auto forward_it = max_it;
        auto reverse_it = max_it;

        while (*(forward_it+1) != 0 && forward_it != cfar_points.end()) ++forward_it;
    
        while (*(reverse_it-1) != 0 && reverse_it != (min_it)) --reverse_it;

        auto window_sz = std::distance(reverse_it, forward_it);
        auto peak_bin = std::distance(cfar_points.begin(), max_it);
        
        float           resolved_bin    { };
        Unit::Azimuth   azimuth         { static_cast<Unit::Azimuth>(fft_msg.azimuth() / steps_per_azimuth) };

        switch (mode) {
            case Subresolution_mode::curve_fit:
                resolved_bin = peak_resolve(power_data, peak_bin, window_sz);
                send_target(resolved_bin, fft_msg.azimuth() / steps_per_azimuth);
                break;
            case Subresolution_mode::centre_of_mass: 
                {
                    auto first_bin = std::distance(cfar_points.begin(), reverse_it);
                    auto window_start = power_data.begin() + first_bin;
                    auto window_end = window_start + window_sz;
                    resolved_bin = first_bin + Utility::centre_of_mass(window_start, window_end);
                    send_target(resolved_bin, azimuth);
                }
                break;
            case Subresolution_mode::centre_of_mass_2d:
                if (azimuth < last_azimuth) {
                    rotations++;
                    if (rotations >= 2) {
                        find_shapes(rotation_data);
                    }
                }

                if (rotations >= 1) {
                    std::vector<Unit::dB> reduced_points(cfar_points.size(), 0.0);
                    auto peaks { 0 };
                    for (auto i { minimum_bin }; i < cfar_points.size(); i++) {
                        if (Utility::essentially_equal(cfar_points[i], 0.0f)) continue;
                        reduced_points[i] = cfar_points[i];
                        peaks++;
                        if (peaks >= max_peaks) break;
                    }

                    rotation_data[azimuth] = reduced_points;
                }

                last_azimuth = azimuth;
                return;
        }
        
    }


    void CFAR_Peak_finder::send_target(float resolved_bin, float resolved_azimuth)
    {
        using Navtech::Utility::stdout_log;
        using Navtech::Utility::endl;
        auto range = to_metre(resolved_bin);
        auto bearing = to_degrees(resolved_azimuth);

        if (std::isinf(range) || range < min_range || range > max_range) return;

        if (target_callback) {
            target_callback({ bearing, range });
        }
    }


    float CFAR_Peak_finder::peak_resolve(
        const std::vector<float>& data,
        std::uint16_t peak_bin,
        std::uint16_t window_sz
    )
    {   
        if (window_sz == 0) return peak_bin;
        const std::uint8_t bins_to_offset   { static_cast<uint8_t>((window_sz - 1) / 2) };
        float x[max_bins_to_operate_on]     { 0.0 };
        float y[max_bins_to_operate_on]     { 0.0 };
        auto index                          { 0 };
        auto startValue                     { 0 - bins_to_offset };

        for (index = 0; index < window_sz; index++) {
            x[index] = static_cast<float>(startValue++);
        }

        auto startBin { peak_bin - bins_to_offset };

        for (index = 0; index < window_sz; index++) {
            y[index] = data[startBin + index];
        }

        float Sx  { };
        float Sx2 { };
        float Sx3 { }; 
        float Sx4 { };
        float x2[max_bins_to_operate_on] { };
        float x3[max_bins_to_operate_on] { };
        float x4[max_bins_to_operate_on] { };

        Vector_maths::scalar_sum(x, window_sz, Sx);
        Vector_maths::scalar_square(x, window_sz, Sx2);
        Vector_maths::vector_cube(x, window_sz, x3);
        Vector_maths::scalar_sum(x3, window_sz, Sx3);
        Vector_maths::vector_square(x, window_sz, x2);
        Vector_maths::vector_multiply(x2, x2, window_sz, x4);
        Vector_maths::scalar_sum(x4, window_sz, Sx4);

        float Sy   { }; 
        float Sxy  { };
        float Sx2y { };
        float xy[max_bins_to_operate_on]  { };
        float x2y[max_bins_to_operate_on] { };

        Vector_maths::scalar_sum(y, window_sz, Sy);
        Vector_maths::vector_multiply(x, y, window_sz, xy);
        Vector_maths::scalar_sum(xy, window_sz, Sxy);
        Vector_maths::vector_multiply(x2, y, window_sz, x2y);
        Vector_maths::scalar_sum(x2y, window_sz, Sx2y);

        float A[4] { Sx2, Sx3, Sx4, Sx2y };
        float B[4] { Sx, Sx2, Sx3, Sxy };
        float C[4] { (float)window_sz, Sx, Sx2, Sy };

        float F = C[0] / A[0];

        for (index = 0; index <= 3; index++) {
            C[index] = C[index] - (F * A[index]);
        }

        F = B[0] / A[0];

        for (index = 0; index <= 3; index++) {
            B[index] = B[index] - (F * A[index]);
        }

        F = C[1] / B[1];

        for (index = 1; index <= 3; index++) {
            C[index] -= F * B[index];
        }

        float b2  { C[3] / C[2] };
        float b1  { (B[3] - B[2] * b2) / B[1] };

        return -b1 / (2 * b2) + startBin + (float)bins_to_offset;
    }


    void CFAR_Peak_finder::find_shapes(const std::vector<std::vector<float>> rotation_data) {

        using namespace Navtech::Utility;

        Shape_finder<float>    shape_finder { minimum_bin };

        auto shape_centres = shape_finder.find_centres(rotation_data);

        for (auto& centre : shape_centres) {
            send_target(
                centre.first,
                centre.second
            );
        }
    }
} // namespace Navtech::Navigation