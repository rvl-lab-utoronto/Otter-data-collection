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
#ifndef BOTAN_POLICY_H
#define BOTAN_POLICY_H

#include <botan/tls_policy.h>

namespace Navtech::Networking::TLS::Botan {

	// ----------------------------------------------------------------------------
    // The TLS_policy provides Navtech-specific implementations
	// for the Botan TLS management policy.
	//
    class TLS_policy : public ::Botan::TLS::Strict_Policy {
	public:
		bool require_cert_revocation_info() const override;
	};

} // namespace Navtech::Networking::TLS::Botan


#endif // BOTAN_POLICY_H