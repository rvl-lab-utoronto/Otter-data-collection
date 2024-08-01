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
#ifndef BOTAN_TLS_ENGINE_H
#define BOTAN_TLS_ENGINE_H

#include <memory>
#include <atomic>

#include <botan/tls_session.h>
#include <botan/auto_rng.h>
#include <botan/tls_callbacks.h>
#include <botan/tls_alert.h>
#include <botan/tls_channel.h>
#include <botan/tls_policy.h>
#include <botan/x509path.h>
#include <botan/ocsp.h>
#include <botan/hex.h>


#include "pointer_types.h"
#include "Port.h"
#include "Message_buffer.h"

#include "Active.h"


namespace Botan {
    class Credentials_Manager;

    namespace TLS { 
        class Strict_Policy; 
        class Session_Manager;
    }
}


namespace Navtech::Networking::TLS::Botan {

    class TLS_services;

    // ----------------------------------------------------------------------------
    // The TLS_engine class encapsulates the TLS server components, and provides
    // an interface for sending/receiving data.
    //
    class Server : public Utility::Active {
    public:
        enum Status { inactive, established, activated };

        Server(TLS_services& tls_services);

        // Ports provide the input/output interface
        //
        using Message_port = Utility::Port<Message_buffer>;
        using Status_port  = Utility::Port<Status>;
        using Error_port   = Utility::Port<int>;

        Message_port encrypt_in  { };
        Message_port encrypt_out { };
        Message_port decrypt_in  { };
        Message_port decrypt_out { };

        Status_port status { };
        Error_port  error  { };

        bool is_ready() const;

        void on_start() override;
        void on_stop()  override;
    
    protected:

    private:
        // ---------------------------------------------------------------------------
        // The Callback object is used by the Botan component 
        // (Server or Client) to route data into/out-of the 
        // Botan object.
        //
        class Callbacks : public ::Botan::TLS::Callbacks {
        public:
            Callbacks(Server& owner);

            void tls_record_received(uint64_t rec_no, const uint8_t buf[], size_t buf_len)      override;
            void tls_emit_data(const uint8_t buf[], size_t buf_len)                             override;
            bool tls_session_established(const ::Botan::TLS::Session& session)                  override;
            void tls_alert(::Botan::TLS::Alert alert)                                           override;
            void tls_session_activated()                                                        override;
        
        private:
            association_to<Server> server;
        };
        // ---------------------------------------------------------------------------

        // External associations
        //
        association_to<TLS_services>   tls_services  { };

        friend class Callbacks;
        Callbacks callbacks;
        
        // TLS::Channel is the base class for the Botan
        // client AND server
        //
        owner_of<::Botan::TLS::Channel> channel { nullptr };

        // Async function implementations
        //
        void do_encrypt(const Message_buffer& buf);
        void do_decrypt(const Message_buffer& buf);

        std::atomic<bool> ready { false };
    };

    
} // namespace Navtech::Networking::TLS::Botan


#endif // BOTAN_TLS_ENGINE_H