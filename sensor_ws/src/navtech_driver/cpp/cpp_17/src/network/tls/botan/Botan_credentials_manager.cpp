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
#include "Botan_credentials_manager.h"


namespace Navtech::Networking::TLS::Botan {
    
    Credentials_manager::Credentials_manager()
    {
    }


    std::vector<::Botan::Certificate_Store*> 
    Credentials_manager::trusted_certificate_authorities(
        const std::string& type     [[maybe_unused]], 
        const std::string& hostname [[maybe_unused]]
    )
    {
        using std::vector;

        vector<::Botan::Certificate_Store*> certificate_stores { };

        if (type == "tls_server") {
            return certificate_stores;
        }
        else {
            for (const auto& ptr : cert_stores) {
                certificate_stores.push_back(ptr.get());
            }
            return certificate_stores;
        }
    }

} // namespace Navtech::Networking::TLS::Botan