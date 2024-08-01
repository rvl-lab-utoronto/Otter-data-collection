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
#include "sdk.h"
#include "Log.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;

namespace Navtech::SDK {

#ifdef _WIN32
    bool wsa_initialised { };

    bool initialise()
    {
        stdout_log.time_format("%T.%ms");
        stdout_log.min_level(Logging_level::debug);

        stdout_log << "SDK initialising..." << endl;

        if (wsa_initialised) return true;

        WSADATA wsa_data { };
        
        int err { WSAStartup(MAKEWORD(2, 2), &wsa_data) };
        
        if (err == 0) {
            stdout_log << "Windows networking started." << endl;
            wsa_initialised = true;
        }
        else {
            stdout_log << Logging_level::error;
            stdout_log << "Failed to start Windows networking!" << endl;
            stdout_log.pop_level();
            wsa_initialised = false;
        }

        return wsa_initialised;
    }


    void shutdown()
    {
        stdout_log << "SDK shutting down..." << endl;
        if (wsa_initialised) WSACleanup();
    }

#else

    bool initialise()
    {
        stdout_log.time_format("%T.%ms");
        stdout_log.min_level(Logging_level::debug);

        stdout_log << "SDK initialising..." << endl;
        return true;
    }


    void shutdown()
    {
        stdout_log << "SDK shutting down..." << endl;
    }

#endif 

} // namespace Navtech::SDK