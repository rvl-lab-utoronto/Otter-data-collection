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
#ifndef BOTAN_SERVER_CREDENTIALS_MANAGER_H
#define BOTAN_SERVER_CREDENTIALS_MANAGER_H

#include <vector>
#include <string>

#include <botan/pubkey.h>
#include <botan/rsa.h>

#include "Botan_credentials_manager.h"
#include "pointer_types.h"


namespace Navtech::Networking::TLS::Botan {

		// ----------------------------------------------------------------------------
		// The Server_credentials_manager provides Navtech-specific certificate 
		// management, for use with the Botan TLS libraries
		//
		class Server_credentials_manager : public Credentials_manager {
		public:
			Server_credentials_manager();
			
			void set_certificate(const std::string& server_cert);
			void set_key(const std::string& server_key, const std::string& passphrase);
			bool load();
		
		protected:
			std::vector<::Botan::X509_Certificate> cert_chain(
				const std::vector<std::string>& algos, 
				const std::string& type, 
				const std::string& hostname) override;

			::Botan::Private_Key* private_key_for(
				const ::Botan::X509_Certificate& certificate, 
				const std::string& type, 
				const std::string& context) override;

		private:
			std::string certificate;
			std::string key;
			std::string passphrase;

			struct Certificate_info {
				std::vector<::Botan::X509_Certificate> certificates;
				shared_owner<::Botan::Private_Key>     key;
			};

			std::vector<Certificate_info> credentials;
		};
		
} // namespace Navtech::Networking::TLS::Botan

#endif // BOTAN_SERVER_CREDENTIALS_MANAGER_H