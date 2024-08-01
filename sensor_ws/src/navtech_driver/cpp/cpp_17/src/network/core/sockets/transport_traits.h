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
#ifndef TRANSPORT_TRAITS_H
#define TRANSPORT_TRAITS_H


namespace Navtech::Networking {

    // ----------------------------------------------------------------------------------
    // The Transport enum is used to select a specific set of traits for 
    // networking classes.
    // For example, a particular protocol may be sent over both TCP and UDP.
    // In such cases a different type of client/server must be used; even though
    // the message type is the same.
    // 
    enum class Transport { 
        tcp,
        udp
    };

} // namespace Navtech::Networking

#endif // TRANSPORT_TRAITS_H