
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
#include <fstream>
#include <filesystem>

#include <botan/auto_rng.h>
#include <botan/ec_group.h>
#include <botan/pkcs8.h>

#include "Botan_key_file.h"

namespace Navtech::Networking::TLS::Botan {

    Key_file::Key_file(const std::filesystem::path& key_filename) : 
        file_name { key_filename }
    {
    }


    bool Key_file::exists() const
    {
        return std::filesystem::exists(file_name);
    }


    Key_file::operator bool() const
    {
        return exists();
    }


    void Key_file::create()
    {
        using std::fstream;
        using std::move;

        ::Botan::AutoSeeded_RNG rand_num_generator { };
        
        private_key.reset(new ::Botan::ECDSA_PrivateKey { rand_num_generator, ::Botan::EC_Group { "secp521r1" } });
        
        fstream key_file { };

        key_file.open(file_name, std::ios::out | std::ios::trunc);
        key_file << ::Botan::PKCS8::PEM_encode(*private_key);
        key_file.close();
    }


    const ::Botan::ECDSA_PrivateKey& Key_file::key() const
    {
        return *private_key;
    }


    std::string Key_file::filename() const
    {
        return file_name.string();
    }

} // namespace Navtech::Networking::TLS::Botan