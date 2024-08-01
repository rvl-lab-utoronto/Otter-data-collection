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

#include <botan/x509self.h>
#include <botan/system_rng.h>
#include <botan/ecdsa.h>

#include "Botan_certificate_file.h"
#include "Botan_key_file.h"

namespace Navtech::Networking::TLS::Botan {

    Certificate_file::Certificate_file(const std::filesystem::path& certificate_filename) :
        file_name { certificate_filename }
    {
    }


    bool Certificate_file::exists() const
    {
        return std::filesystem::exists(file_name);
    }


    Certificate_file::operator bool() const
    {
        return exists();
    }


    std::string Certificate_file::filename() const
    {
        return file_name.string();
    }


    void Certificate_file::create(const Key_file& key_file)
    {
        ::Botan::X509_Cert_Options options("", 5000 * 24 * 60 * 60);

        options.common_name     = "epu.navtechradar.co.uk";
        options.country         = "GB";
        options.organization    = "Navtech Radar Ltd";
        options.email           = "navtech.admin@navtechradar.com";
        options.more_dns.push_back("127.0.0.1");

        auto certificate = ::Botan::X509::create_self_signed_cert(options, key_file.key(), "SHA-256", ::Botan::system_rng());
        
        std::fstream certificate_file { };
        certificate_file.open(file_name, std::ios::out | std::ios::trunc);
        certificate_file << certificate.PEM_encode();
        certificate_file.close();
    }


} // namespace Navtech::Networking::TLS::Botan