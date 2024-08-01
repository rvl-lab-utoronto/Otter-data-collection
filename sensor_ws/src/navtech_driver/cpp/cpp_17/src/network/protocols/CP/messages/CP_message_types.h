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
#ifndef CP_MESSAGE_TYPES_H
#define CP_MESSAGE_TYPES_H

#include <cstdint>

namespace Navtech::Networking::CP_protocol {

    enum class Type : std::uint8_t {
        invalid                                             = 0,
        configuration                                       = 1,
        health                                              = 2,
        track_update                                        = 3,
        keep_alive                                          = 4,
        system_restart                                      = 5,
        contour_update                                      = 6,
        tracker_distribution_update                         = 7,
        tracker_save_clutter_map                            = 8,
        tracker_playback_command                            = 9,
        restart_tracker                                     = 10,
        playback_file_list_update                           = 11,
        playback_file_request                               = 12,
        playback_file_list_response                         = 13,
        restart_track_engine                                = 14,
        shutdown_track_engine                               = 15,
        takeover_backup_track_engine                        = 16,
        radar_details                                       = 17,
        video_server_command                                = 18,
        restart_shutdown                                    = 19,
        version_details                                     = 20,
        cryptographic_failure                               = 21,
        request_public_key                                  = 100,
        public_key                                          = 101,
        reset_public_key                                    = 102,
        session_key                                         = 103,
        request_video_server_startup_parameters             = 104,
        video_server_startup_parameters                     = 105,
        major_incident                                      = 106,
        simulated_path_config                               = 107,
        playback_track_command                              = 108,
        reboot_tracker                                      = 109,
        playback_info                                       = 110,
        static_target_detection_capture_baseline            = 111,
        static_target_detection_capture_threshold_request   = 112,
        static_target_detection_capture_threshold_response  = 113,
        disable_encryption                                  = 114,
        CP_licence_request                                  = 115,
        CP_licence_response                                 = 116,
        CP_licence_free                                     = 117,
        CP_licence_status                                   = 118
    };

} // namespace Navtech::Networking::CP_protocol

#endif // CP_MESSAGE_TYPES_H