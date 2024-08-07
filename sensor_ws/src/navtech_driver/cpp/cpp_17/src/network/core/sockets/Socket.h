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
#ifndef SOCKET_H
#define SOCKET_H

#ifdef __linux__
#include <arpa/inet.h>
#include <unistd.h>
#elif _WIN32
#include <WinSock2.h>
#pragma comment(lib, "Ws2_32.lib")
#endif

#include <cstdint>
#include <cstddef>
#include <vector>
#include <system_error>
#include <mutex>

#include "Endpoint.h"
#include "Time_utils.h"

namespace Navtech::Networking {

    // -----------------------------------------------------------------------------
    // Socket is the base class for all socket-type connections.
    // It defines the common elements of all sockets.
    // Note: as Sockets will not be used in substitution heirarchies
    // the Socket class does not define any virtual functions
    //
    class Socket {
    public:
    #ifdef __linux__
        using Native_handle = int;
        static constexpr Native_handle invalid_handle { -1 };

    #elif _WIN32
        using Native_handle = SOCKET;
        static constexpr Native_handle invalid_handle { 0 };
    #endif
    
        enum Read_mode { consume, peek };

        Socket() = default;
        ~Socket();

        // Copy / move policy
        //
        Socket(const Socket&)               = delete;
        Socket& operator=(const Socket&)    = delete;
        Socket(Socket&&);
        Socket& operator=(Socket&&);

        bool is_open() const noexcept;
        void close() noexcept;
        bool valid() noexcept;
        operator bool() noexcept;
        Native_handle native_handle() const noexcept;
        
        const Endpoint& local_endpoint() const;

        void tx_timeout(const Time::Duration& timeout);
        Time::Duration tx_timeout() const;

        void rx_timeout(const Time::Duration& timeout);
        Time::Duration rx_timeout() const;

        void tx_buffer(std::size_t sz);
        void rx_buffer(std::size_t sz);

        void keep_alive(bool enable);
        bool keep_alive() const;

        void reuse_address();
        void linger();

        // Diagnostic counters
        //
        std::uint64_t bytes_sent() const    { return sent; }
        std::uint64_t bytes_read() const    { return recvd; }

    protected:
        // Clients cannot create new Sockets from a handle
        //
        Socket(Native_handle socket_handle);
        Socket(Native_handle socket_handle, const Endpoint& endpt);

        void local_endpoint(const Endpoint& endpt);

        void ready();
        bool is_valid() const;
        int  last_error() const;

        // Diagnostic counters
        //
        void bytes_sent(std::uint64_t num_bytes)    { sent += num_bytes; }
        void bytes_read(std::uint64_t num_bytes)    { recvd += num_bytes; }

    private:
        Native_handle handle      { invalid_handle };
        Endpoint      local_endpt { };

        mutable std::mutex mtx { };
        bool open { false };

        void set_options();

        // Diagnostic counters
        //
        std::uint64_t sent  { };
        std::uint64_t recvd { };
    };

} // namespace Navtech::Networking

#endif // SOCKET_H