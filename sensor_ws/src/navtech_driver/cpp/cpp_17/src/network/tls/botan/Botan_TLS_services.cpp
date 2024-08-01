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
#include <exception>

#include "Botan_TLS_services.h"
#include "Botan_key_file.h"
#include "Botan_certificate_file.h"
#include "Log.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;


namespace Navtech::Networking::TLS::Botan {

    // Meyers Singleton
    //
    TLS_services& TLS_services::make()
    {
        static TLS_services instance { };
        return instance;
    }


    TLS_services::TLS_services() 
    {
        try {
            init_policy();
            init_session_manager();
            load_credentials();
            
            active = true;
            stdout_log << "TLS services loaded." << endl;
        }
        catch (std::exception& ex) {
            stdout_log << "TLS services failed to initialise [" << ex.what() << "]" << endl;
            active = false;
        }
    }


    void TLS_services::init_policy()
    {
        policy_ptr = allocate_owned<Policy>();

        if (policy_ptr == nullptr) {
            throw std::domain_error { "Failed to allocate TLS policy" };
        }
    }


    void TLS_services::init_session_manager()
    {
        session_mgr_ptr = allocate_owned<Session_manager>(rng());

        if (policy_ptr == nullptr) { 
            throw std::domain_error { "Failed to allocate session manager" };
        }
    }


    void TLS_services::load_credentials()
    {
        if (!config.load()) {
            config.create();
        }

        Key_file         key_file  { config.key_filename };
        Certificate_file cert_file { config.certificate_filename };

        if (!key_file)  key_file.create();
        if (!cert_file) cert_file.create(key_file);

        cred_mgr_ptr = allocate_owned<Credentials_manager>();
    
        if (cred_mgr_ptr == nullptr) {
            throw std::domain_error { "Failed to allocate credentials manager" };
        }
    
        cred_mgr_ptr->set_certificate(config.certificate_filename.string());
        cred_mgr_ptr->set_key(config.key_filename.string(), config.passphrase);

        if (!cred_mgr_ptr->load()) {
            throw std::domain_error { "Could not load credentials" };
        }
    }


    TLS_services::Credentials_manager& TLS_services::credentials_mgr() const
    {
        return *cred_mgr_ptr;
    }


    TLS_services::Session_manager& TLS_services::session_mgr() const
    {
        return *session_mgr_ptr;
    }


    TLS_services::Policy& TLS_services::policy() const
    {
        return *policy_ptr;
    }   


    TLS_services::Random_number_generator& TLS_services::rng() const
    {
        return ::Botan::system_rng();
    }


    bool TLS_services::ready() const
    {
        return active;
    }


    TLS_services::operator bool() const
    {
        return ready();
    }

} // namespace Navtech::Networking::TLS::Botan