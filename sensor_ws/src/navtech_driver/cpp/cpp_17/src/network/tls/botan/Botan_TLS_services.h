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
#ifndef BOTAN_TLS_SERVICES_H
#define BOTAN_TLS_SERVICES_H


#include <memory>

#include <botan/system_rng.h>
#include <botan/tls_session_manager.h>

#include "Botan_server_credentials_manager.h"
#include "Botan_policy.h"
#include "TLS_config.h"
#include "pointer_types.h"

namespace Navtech::Networking::TLS::Botan {

    // ---------------------------------------------------------------------------
    // TLS_services represents the shared/static TLS facilities required to
    // support individual TLS sessions (as represented by the TLS_engine class)
    // TLS_services provides an abstract interface for facilities such as
    // - Certificate management
    // - Credentials management
    // - Reconnection state
    // - etc.  
    //
    // There should only be one instance of this class in the system.  However,
    // lazy-instantiation is also desirable, so rather than constructing the
    // TLS_services as a static object, it is a Singleton (I feel dirty now)
    //
    class TLS_services {
    public:
        using Credentials_manager       = Server_credentials_manager;
        using Policy                    = TLS_policy;
        using Session_manager           = ::Botan::TLS::Session_Manager_In_Memory;
        using Random_number_generator   = ::Botan::RandomNumberGenerator;

        static TLS_services& make();

        Credentials_manager&     credentials_mgr() const;
        Session_manager&         session_mgr()     const;
        Policy&                  policy()          const;
        Random_number_generator& rng()             const;

        bool ready() const;
        operator bool() const;

    protected:
        TLS_services();

    private:
        TLS::Configuration            config          { };
        owner_of<Credentials_manager> cred_mgr_ptr    { nullptr };
        owner_of<Session_manager>     session_mgr_ptr { nullptr };
        owner_of<Policy>              policy_ptr      { nullptr };

        bool active { false };

        void init_policy();
        void init_session_manager();
        void load_credentials();
    };

} // namespace Navtech::Networking::TLS::Botan


#endif // BOTAN_TLS_SERVICES_H