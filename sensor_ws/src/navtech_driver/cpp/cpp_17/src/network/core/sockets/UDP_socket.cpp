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
#ifdef _WIN32
    #include <Windows.h>
    #include <WinSock2.h>
    #include <Ws2tcpip.h>   // needed for ip_mreq definition for multicast
#else
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <time.h>
#endif

#include "UDP_socket.h"
#include "socket_exceptions.h"
#include "Log.h"

using std::size_t;
using std::vector;
using std::uint8_t;
using std::int32_t;
using std::system_error;
using std::error_code;
using std::system_category;

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;


namespace Navtech::Networking {

    UDP_socket::UDP_socket() : 
        Socket          { ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP) }
    {
    }


    UDP_socket::UDP_socket(const Endpoint& local_endpt) :
        Socket          { ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP) }
    {
        bind_to(local_endpt);
    }
    
    
    UDP_socket::UDP_socket(const Endpoint& local_endpt, const Endpoint& remote_endpt) :
        Socket          { ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP) }
    {
        bind_to(local_endpt);
        remote_endpoint(remote_endpt);
    }


    UDP_socket::UDP_socket(const Endpoint& local_endpt, const Endpoint& remote_endpt, std::uint8_t TTL) :
        Socket          { ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP) },
        ttl             { TTL }
    {
        bind_to(local_endpt);
        remote_endpoint(remote_endpt);
    }


    UDP_socket::UDP_socket(Socket::Native_handle socket_handle, const Endpoint& local_endpt) :
        Socket { socket_handle, local_endpt }
    {
        bind_to(local_endpt);
    }


    UDP_socket::UDP_socket(UDP_socket&& src) :
        Socket          { std::move(src) },
        remote_endpt    { std::move(src.remote_endpt) }
    {
    }     


    UDP_socket& UDP_socket::operator=(UDP_socket&& rhs)
    {
        Socket::operator=(std::move(rhs));
        remote_endpt = std::exchange(rhs.remote_endpt, Endpoint::null());

        return *this;
    }


    std::size_t UDP_socket::send(const Message_buffer& buffer)
    {
        if (!is_open())    return { };
        if (!remote_endpt) return { };

        auto addr = remote_endpt.to_sockaddr();
        int32_t flags { };

    #ifdef __linux__
        flags |= MSG_NOSIGNAL;

        auto result = ::sendto(
            native_handle(),
            buffer.data(),
            buffer.size(),
            flags,
            reinterpret_cast<sockaddr*>(&addr),
            sizeof(addr)
        );
    #elif _WIN32
        auto result = ::sendto(
            native_handle(),
            (char*)buffer.data(),
            (int)buffer.size(),
            flags,
            reinterpret_cast<sockaddr*>(&addr),
            sizeof(addr)
        );
    #endif

        if (result < 0)  throw system_error { last_error(), system_category(), "UDP Socket receive" };
        if (result == 0) throw client_shutdown { };

        // Diagnostic tracking
        //
        bytes_sent(result);

        return result;
    }


    std::size_t UDP_socket::send(Message_buffer&& buffer)
    {
        // Take ownership to maintain move semantics
        //
        Message_buffer buf { std::move(buffer) };
        return send(buf);
    }


    std::pair<IP_address, Message_buffer> UDP_socket::receive()
    {
        if (!is_open()) return { };

        int flags       { };
        int num_bytes   { 65536 };

        sockaddr_in sender_addr { };
        socklen_t   addr_len    { sizeof(sender_addr) };

        recv_buffer.resize(num_bytes);

        #ifdef __linux__
        recv_buffer.resize(num_bytes);

        auto result = ::recvfrom(
            native_handle(),
            recv_buffer.data(),
            num_bytes,
            flags,
            reinterpret_cast<struct sockaddr*>(&sender_addr),
            &addr_len
        );
    #elif _WIN32
        auto result = ::recvfrom(
            native_handle(),
            (char*)recv_buffer.data(),
            (int)num_bytes,
            flags,
            reinterpret_cast<struct sockaddr*>(&sender_addr),
            &addr_len
        );
    #endif

        if (result < 0)  throw system_error    { last_error(), system_category(), "UDP Socket receive" };
        if (result == 0) throw client_shutdown { };

        recv_buffer.resize(result);

        // Diagnostic tracking
        //
        bytes_read(result);

        return std::make_pair(sender_addr, std::move(recv_buffer));
    }


    void UDP_socket::bind_to(const Endpoint& endpt)
    {
        local_endpoint(endpt);

        auto addr = local_endpoint().to_sockaddr();

        auto result = ::bind(
            native_handle(), 
            reinterpret_cast<struct sockaddr*>(&addr), 
            sizeof(addr)
        );

        if (result < 0) {
            stdout_log << "UDP bind returns: " << result << " - " << last_error() << endl;
            throw system_error { last_error(), system_category(), "UDP Socket bind" };
        }
        ready();
    }


    Endpoint UDP_socket::peer() const noexcept
    {
        sockaddr    addr            { };
		socklen_t   size            { sizeof(addr) };
        Endpoint    peer_endpoint { };

		auto result = ::getpeername(native_handle(), &addr, &size);
	
        if (result == 0) {
            peer_endpoint = Endpoint { addr, local_endpoint().port };
        }

		return peer_endpoint;
    }


    void UDP_socket::remote_endpoint(const Endpoint& endpt)
    {
        remote_endpt = endpt;
        if (is_multicast_address(endpt)) enable_multicast();

        ready();
    }
    

    bool UDP_socket::is_multicast_address(const Endpoint& endpt) const
    {
        std::uint32_t low_addr  = "224.0.0.1"_ipv4;
        std::uint32_t high_addr = "239.255.255.255"_ipv4;
        return ((endpt.ip_address >= low_addr) && (endpt.ip_address <= high_addr));
    }


    void UDP_socket::enable_multicast()
    {
        int     result { };
        ip_mreq mreq   { };

        mreq.imr_multiaddr.s_addr = remote_endpt.ip_address.to_network_endian();
        mreq.imr_interface.s_addr = local_endpoint().ip_address.to_network_endian();

        reuse_address();

        result = setsockopt(
            native_handle(), 
            IPPROTO_IP, 
            IP_ADD_MEMBERSHIP, 
            reinterpret_cast<char*>(&mreq), 
            sizeof(mreq)
        );

        if (result < 0) throw system_error { errno, system_category(), "UDP Socket add group membership" };

    #ifdef __linux__
        int loop { 0 };
    #elif _WIN32
        const char loop { 0 };
    #endif

        result = setsockopt(
            native_handle(),
            IPPROTO_IP,
            IP_MULTICAST_LOOP,
            &loop,
            sizeof(loop)
        );

        if (result < 0) throw system_error { errno, system_category(), "UDP Socket multicast loopback" };

    #ifdef __linux__
        result = setsockopt(
            native_handle(),
            IPPROTO_IP,
            IP_MULTICAST_TTL,
            &ttl,
            sizeof(ttl)
        );
    #elif _WIN32
        result = setsockopt(
            native_handle(),
            IPPROTO_IP,
            IP_MULTICAST_TTL,
            (char*)&ttl,
            sizeof(ttl)
        );
    #endif

        if (result < 0) throw system_error { errno, system_category(), "UDP Socket multicast TTL" };
    }




} // namespace Navtech::Networking