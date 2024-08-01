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
#ifndef BOTAN_CREDENTIALS_MANAGER_H
#define BOTAN_CREDENTIALS_MANAGER_H

#include <string>
#include <vector>

#include <botan/auto_rng.h>
#include <botan/pkcs8.h>
#include <botan/credentials_manager.h>
#include <botan/x509self.h>
#include <botan/data_src.h>

#include "pointer_types.h"


namespace Navtech::Networking::TLS::Botan {

	// ----------------------------------------------------------------------------
	// The Credentials_manager provides common certificate management facilities
	// for use with the Botan TLS library.
	// This class is designed to be used only as a base class for either a server- 
	// or client- credentials-management class.
	//
	class Credentials_manager : public ::Botan::Credentials_Manager {
	public:
		Credentials_manager();

	protected:
		std::vector<::Botan::Certificate_Store*> 
		trusted_certificate_authorities(const std::string& type, const std::string& hostname) override;

	protected:
		::Botan::AutoSeeded_RNG rand_num_generator { };

		using Cert_store_container = std::vector<owner_of<::Botan::Certificate_Store>>;

		Cert_store_container cert_stores { };
	};

} // namespace Navtech::Networking::TLS::Botan

#endif // BOTAN_CREDENTIALS_MANAGER_H
