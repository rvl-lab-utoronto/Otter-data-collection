
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
#include <iterator>

#include <botan/tls_session_manager.h>
#include <botan/credentials_manager.h>
#include <botan/tls_policy.h>
#include <botan/tls_server.h>
#include <botan/tls_client.h>

#include "Botan_TLS_server.h"
#include "Botan_TLS_services.h"
#include "Log.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;


namespace Navtech::Networking::TLS::Botan {

    // ------------------------------------------------------------------------------------------
    // Server
    //
    Server::Server(TLS_services& tls_srvc) :
        Active          { "Botan TLS server" },
        tls_services    { associate_with(tls_srvc) },
        callbacks       { *this }
    {
        // NOTE:  
        // The TLS component is only fully open
        // once the TLS handshake has completed.
        //
        encrypt_in.on_receive(
            [this](Message_buffer& buf) 
            { 
                async_call(&Server::do_encrypt, this, buf); 
            }
        );

        decrypt_in.on_receive(
            [this](Message_buffer& buf) 
            { 
                async_call(&Server::do_decrypt, this, buf); 
            }
        );
    }


    void Server::on_start()
    {
        channel = allocate_owned<::Botan::TLS::Server>(
            callbacks,
            tls_services->session_mgr(),
            tls_services->credentials_mgr(),
            tls_services->policy(),
            tls_services->rng()
        );
    }
    
    
    void Server::on_stop() 
    {
        encrypt_in.disconnect();
        decrypt_in.disconnect();
        encrypt_out.disconnect();
        encrypt_out.disconnect();
    }


    void Server::do_encrypt(const Message_buffer& buf)
    {
        if (!channel) return;

        channel->send(buf);
    }


    void Server::do_decrypt(const Message_buffer& buf)
    {
        if (!channel) return;

        channel->received_data(buf);
    }


    bool Server::is_ready() const
    {
        return ready;
    }


    // ------------------------------------------------------------------------------------------
    // TLS callbacks
    //
    Server::Callbacks::Callbacks(Server& owner) :
        server { &owner }
    {
    }

    // Called once for each application_data record which is received, with the matching
    // (TLS level) record sequence number.
    // As with tls_emit_data, the array will be overwritten sometime after the callback 
    // returns, so a copy should be made if needed.
    //
    void Server::Callbacks::tls_record_received(uint64_t rec_no [[maybe_unused]], const uint8_t buf[], size_t buf_len)
    {
        using std::memcpy;
        using std::move;
    
        Message_buffer msg { };
        msg.resize(buf_len);
        memcpy(msg.data(), buf, buf_len);

        server->decrypt_out.post(move(msg));
    }

    // The TLS stack requests that all bytes of data be queued up to send to the counterparty. 
    // After this function returns, the buffer containing data will be overwritten, so a copy 
    // of the input must be made if the callback cannot send the data immediately.
    //
    void Server::Callbacks::tls_emit_data(const uint8_t buf[], size_t buf_len)
    {
        using std::memcpy;
        using std::move;
    
        Message_buffer msg { };
        msg.resize(buf_len);
        memcpy(msg.data(), buf, buf_len);

        server->encrypt_out.post(move(msg));
    }


    // Called whenever a negotiation completes. This can happen more than once on any connection, 
    // if renegotiation occurs. The session parameter provides information about the session 
    // which was just established.
    // If this function returns false, the session will not be cached for later resumption.
    //
    bool Server::Callbacks::tls_session_established(const ::Botan::TLS::Session& session [[maybe_unused]])
    {
        stdout_log << "TLS session established. "
                   << "Version [ " << session.version().to_string() + "] " 
                   << " Cypher suite [" << session.ciphersuite().to_string() << "]"
                   << endl;  
        
        server->status.post(established);
        return true;
    }


    // Called when an alert is received from the peer. Note that alerts received 
    // before the handshake is complete are not authenticated and could have been 
    // inserted by a MITM attacker.
    //
    void Server::Callbacks::tls_alert(::Botan::TLS::Alert alert)
    {
        stdout_log << "TLS session error [" << alert.type_string() << "]" << endl;
        
        server->ready = false;
        server->error.post(static_cast<int>(alert.type()));
    }


    // This is called when the session is activated, that is once it is possible 
    // to send or receive data on the channel. In particular it is possible for 
    // an implementation of this function to perform an initial write on the channel.
    //
    void Server::Callbacks::tls_session_activated()
    {
        stdout_log << "TLS session activated. Ready for comms." << endl;

        // TLS is only active once the TLS handshake
        // is fully complete.
        //
        server->ready = true;
        server->status.post(activated);
    }

} // namespace Navtech::Networking::TLS::Botan