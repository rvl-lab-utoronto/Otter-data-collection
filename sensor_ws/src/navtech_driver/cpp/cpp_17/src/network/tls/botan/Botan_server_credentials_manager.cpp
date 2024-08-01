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
#include <algorithm>
#include <string>

#include "Botan_server_credentials_manager.h"
#include "Log.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;


namespace Navtech::Networking::TLS::Botan {

    Server_credentials_manager::Server_credentials_manager() : 
        Credentials_manager { }
    {
    }

    void Server_credentials_manager::set_certificate(const std::string& server_cert)
    {
        certificate = server_cert;
    }
    

    void Server_credentials_manager::set_key(const std::string& in_key, const std::string& in_passphrase)
    {
        key        = in_key;
        passphrase = in_passphrase;
    }


    bool Server_credentials_manager::load()
    {
        using std::move;
        using std::string;

        Certificate_info certificate_info;
        
        try {
            if (passphrase.empty()) {
                certificate_info.key.reset(
                    ::Botan::PKCS8::load_key(key, rand_num_generator)
                );
            } 
            else {
                certificate_info.key.reset(
                    ::Botan::PKCS8::load_key(key, rand_num_generator, passphrase)
                );
            }
        }
        catch (const std::exception& e) {
            stdout_log << "TLS - Server credentials load failed: " << e.what() << endl;
    
            return false;
        }
        
        try {
            ::Botan::DataSource_Stream input { certificate };
            
            while (!input.end_of_data()) {
                try {
                    certificate_info.certificates.push_back(::Botan::X509_Certificate { input });
                }
                catch (const std::exception& e) {                    
                    if (input.end_of_data()) {
                        break;
                    }

                    stdout_log << "TLS - failed to load certificate: " << e.what() << endl;
                
                    return false;
                }            
            }
        }
        catch (const std::exception& e) {
            stdout_log << "TLS - failed to load certificate: " << e.what() << endl;
            
            return false;
        }			

        if (certificate_info.certificates.size() == 0) {
            stdout_log << "TLS - failed to load any certificates! " << endl;
            
            return false;
        }

        credentials.push_back(move(certificate_info));
        return true;
    }


    std::vector<::Botan::X509_Certificate> Server_credentials_manager::cert_chain(
        const std::vector<std::string>& algos, 
        const std::string& type     [[maybe_unused]], 
        const std::string& hostname [[maybe_unused]])
    {
        using std::find;
        using std::begin;
        using std::end;

        for (const auto& credential : credentials) {
            if (find(begin(algos), end(algos), credential.key->algo_name()) == end(algos)) {
                continue;
            }

            if (hostname != "" && !credential.certificates[0].matches_dns_name(hostname)) {
                continue;
            }

            return credential.certificates;
        }

        return std::vector<::Botan::X509_Certificate> { };
    }


    ::Botan::Private_Key* Server_credentials_manager::private_key_for(
        const ::Botan::X509_Certificate& certificate, 
        const std::string& type    [[maybe_unused]], 
        const std::string& context [[maybe_unused]])
    {
        for (auto const& credential : credentials) {
            if (certificate == credential.certificates[0]) {
                return credential.key.get();
            }
        }

        return nullptr;
    }

} // namespace Navtech::networking::TLS::Botan