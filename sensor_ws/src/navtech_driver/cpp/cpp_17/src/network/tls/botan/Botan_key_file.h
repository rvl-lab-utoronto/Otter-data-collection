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
#ifndef BOTAN_KEY_FILE_H
#define BOTAN_KEY_FILE_H

#include <string>
#include <filesystem>
#include <memory>
#include <botan/ecc_key.h>
#include <botan/ecdsa.h>

#include "pointer_types.h"


namespace Navtech::Networking::TLS::Botan {

    // ----------------------------------------------------------------------------
    // The Key_file class provides an interface for the creation, loading and
    // storing of Botan encryption keys.
    //
    class Key_file {
    public:
        Key_file(const std::filesystem::path& key_filename);
        bool exists() const;
        operator bool() const;
        void create();
        const ::Botan::ECDSA_PrivateKey& key() const;
        std::string filename() const;

    private:
        std::filesystem::path file_name { };

        owner_of<::Botan::ECDSA_PrivateKey> private_key { nullptr };
    };

} // namespace Navtech::Networking::TLS::Botan


#endif  // BOTAN_KEY_FILE_H