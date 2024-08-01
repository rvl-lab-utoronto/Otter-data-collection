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
#ifdef __linux__
#include <arpa/inet.h>
#include <netinet/tcp.h>
#elif _WIN32
#include <Windows.h>
#include <WinSock2.h>
#endif

#include <system_error>

#include "TCP_socket.h"
#include "net_conversion.h"
#include "Memory_types.h"
#include "socket_exceptions.h"

using std::size_t;
using std::vector;
using std::uint8_t;
using std::int32_t;
using std::system_error;
using std::error_code;
using std::system_category;
using namespace Navtech::Unit;


namespace Navtech::Networking {

    // -------------------------------------------------------------------------
    // Construction / destruction
    //
    TCP_socket::TCP_socket() : Socket { ::socket(AF_INET, SOCK_STREAM, 0) }
    {
        recv_buffer.reserve(64_kB);
    }


    TCP_socket::TCP_socket(Socket::Native_handle socket_handle, const Endpoint& endpt) :
        Socket { socket_handle, endpt }
    {
    }


    // -------------------------------------------------------------------------
    // Read/write interface
    //
    size_t TCP_socket::send(const vector<uint8_t>& buffer)
    {
        int32_t flags { };
    #ifdef __linux__
        flags |= MSG_NOSIGNAL;

        auto result = ::send(
            native_handle(),
            buffer.data(),
            buffer.size(),
            flags
        );
    #elif _WIN32
        auto result = ::send(
            native_handle(),
            (char*)buffer.data(),
            (int)buffer.size(),
            flags
        );
    #endif

        if (result < 0)  throw system_error { last_error(), system_category(), "TCP Socket send" };
        if (result == 0) throw client_shutdown { };

        // Diagnostic tracking
        //
        bytes_sent(result);

        return result;
    }


    size_t TCP_socket::send(vector<uint8_t>&& buffer)
    {
        // Take ownership for send
        //
        vector<uint8_t> send_buffer { std::move(buffer) };
        return send(send_buffer);
    }


    vector<uint8_t> TCP_socket::receive(size_t num_bytes)
    {
        return receive(num_bytes, Socket::Read_mode::consume);
    }


    vector<uint8_t> TCP_socket::receive(size_t num_bytes, Read_mode mode)
    {
        int32_t flags { };
        if (mode == Socket::Read_mode::peek) flags |= MSG_PEEK;

        recv_buffer.resize(num_bytes);

        uint8_t* insert_ptr      { recv_buffer.data() };
        size_t   bytes_remaining { num_bytes };
        size_t   bytes_in        { 0 };

        do {
            if (!is_open()) return { };

        #ifdef __linux__
            auto result = ::recv(
                native_handle(),
                insert_ptr,
                bytes_remaining,
                flags
            );
        #elif _WIN32
            auto result = ::recv(
                native_handle(),
                (char*)insert_ptr,
                (int)bytes_remaining,
                flags
            );
        #endif

            if (result < 0)  throw system_error    { last_error(), system_category(), "TCP Socket receive" };
            if (result == 0) throw client_shutdown { };

            bytes_in   += result;
            insert_ptr += bytes_in;
            bytes_remaining = num_bytes - bytes_in;

            // Diagnostic tracking
            //
            bytes_read(result);

        } while (bytes_remaining > 0);

        return recv_buffer;
    }


    vector<uint8_t> TCP_socket::receive_some(size_t num_bytes)
    {
        return receive_some(num_bytes, Socket::Read_mode::consume);
    }


    vector<uint8_t> TCP_socket::receive_some(size_t num_bytes, Read_mode mode)
    {
        int32_t flags { };
        if (mode == Socket::Read_mode::peek) flags |= MSG_PEEK;

        recv_buffer.resize(num_bytes);

        if (!is_open()) return { };

    #ifdef __linux__
        auto result = ::recv(
            native_handle(),
            recv_buffer.data(),
            num_bytes,
            flags
        );
    #elif _WIN32
        auto result = ::recv(
            native_handle(),
            (char*)recv_buffer.data(),
            num_bytes,
            flags
        );
    #endif

        if (result < 0)  throw system_error    { last_error(), system_category(), "TCP Socket receive" };
        if (result == 0) throw client_shutdown { };

        recv_buffer.resize(result);

        // Diagnostic tracking
        //
        bytes_read(result);

        return recv_buffer;
    }


    // -------------------------------------------------------------------------
    // Connection interface
    //
    void TCP_socket::bind_to(const Endpoint& endpt)
    {
        // Don't allow binding to port 0; event though this is allowed
        // by Posix sockets
        //
        if (endpt.port == 0) throw system_error { EFAULT, system_category(), "TCP Socket bind" };

        auto addr = endpt.to_sockaddr();

        auto result = ::bind(
            native_handle(), 
            reinterpret_cast<struct sockaddr*>(&addr), 
            sizeof(addr)
        );

        if (result < 0) throw system_error { last_error(), system_category(), "TCP Socket bind" };

        local_endpoint(endpt);
        ready();
    }

    Endpoint TCP_socket::peer() const
    {
        if (!is_open()) return Endpoint::null();
        
        struct sockaddr addr            { };
		socklen_t       size            { sizeof(addr) };
        Endpoint        client_endpoint { };

		auto result = ::getpeername(native_handle(), &addr, &size);
	
        if (result < 0) throw system_error { last_error(), system_category(), "TCP Socket peername" };
        
		return Endpoint { addr, local_endpoint().port };
    }


    void TCP_socket::listen(uint8_t max_connections)
    {
        if (!is_open()) throw system_error { EBADF, system_category(), "TCP Socket listen" };

        auto result = ::listen(
            native_handle(), 
            max_connections
        );

        if (result < 0) throw system_error { last_error(), system_category(), "TCP Socket listen" };
    }


    TCP_socket TCP_socket::accept()
    {
        struct sockaddr addr { };
		socklen_t       size { sizeof(addr) };

        auto result = ::accept(
            native_handle(), 
            reinterpret_cast<struct sockaddr*>(&addr), 
            reinterpret_cast<socklen_t*>(&size)
        );

        if (result < 0) throw system_error { last_error(), system_category(), "TCP Socket accept" };

        return { result, Endpoint { addr } };
    }


    void TCP_socket::connect_to(const Endpoint& endpt)
    {
        auto addr = endpt.to_sockaddr();

        auto result = ::connect(
            native_handle(), 
            reinterpret_cast<struct sockaddr*>(&addr), 
            sizeof(addr)
        );

        if (result < 0) throw system_error { last_error(), system_category(), "TCP Socket connect" };

        local_endpoint(endpt);
        ready();
    }


    // --------------------------------------------------------------------------------------------------
    // TCP-specific configuration.
    // NOTE - Keep-alive is not supported on Windows
    //
#ifdef __linux__
    void TCP_socket::keep_alive(bool enable)
    {
        int on          { enable ? 1 : 0 };
        socklen_t sz    { static_cast<socklen_t>(sizeof(int)) };

        auto result = ::setsockopt(
            native_handle(),
            SOL_SOCKET, 
            SO_KEEPALIVE, 
            reinterpret_cast<void*>(&on), 
            sz
        );

        if (result < 0) throw system_error { errno, system_category(), "TCP Socket keep-alive enable"};
    }


    bool TCP_socket::keep_alive() const
    {
        int is_enabled  { };
        socklen_t sz    { static_cast<socklen_t>(sizeof(int)) };

        ::getsockopt(
            native_handle(),
            SOL_SOCKET,
            SO_KEEPALIVE,
            reinterpret_cast<void*>(&is_enabled),
            &sz
        );

        return is_enabled;
    }


    void TCP_socket::keep_alive_idle(const Time::Duration& idle_time)
    {
        int idle        { static_cast<int>(idle_time.seconds()) };
        socklen_t sz    { static_cast<socklen_t>(sizeof(int)) };

        auto result = ::setsockopt(
            native_handle(),
            SOL_TCP, 
            TCP_KEEPIDLE, 
            reinterpret_cast<void*>(&idle), 
            sz
        );

        if (result < 0) throw system_error { errno, system_category(), "TCP Socket keep-alive idle time"};
    }


    Time::Duration TCP_socket::keep_alive_idle() const
    {
        int idle        { };
        socklen_t sz    { static_cast<socklen_t>(sizeof(int)) };

        ::getsockopt(
            native_handle(),
            SOL_TCP, 
            TCP_KEEPIDLE,
            reinterpret_cast<void*>(&idle),
            &sz
        );

        return Time::to_sec_duration(idle);
    }


    void TCP_socket::keep_alive_probes(unsigned num_probes)
    {
        int probes      { static_cast<int>(num_probes) };
        socklen_t sz    { static_cast<socklen_t>(sizeof(int)) };

        auto result = ::setsockopt(
            native_handle(),
            SOL_TCP, 
            TCP_KEEPCNT, 
            reinterpret_cast<void*>(&probes), 
            sz
        );

        if (result < 0) throw system_error { errno, system_category(), "TCP Socket keep-alive probe count"};
    }


    unsigned TCP_socket::keep_alive_probes() const
    {
        int probes      { };
        socklen_t sz    { static_cast<socklen_t>(sizeof(int)) };

        ::getsockopt(
            native_handle(),
            SOL_TCP, 
            TCP_KEEPCNT,
            reinterpret_cast<void*>(&probes),
            &sz
        );

        return static_cast<unsigned>(probes);
    }


    void TCP_socket::keep_alive_interval(const Time::Duration& probe_interval)
    {
        int interval    { static_cast<int>(probe_interval.seconds()) };
        socklen_t sz    { static_cast<socklen_t>(sizeof(int)) };

        auto result = ::setsockopt(
            native_handle(),
            SOL_TCP, 
            TCP_KEEPINTVL, 
            reinterpret_cast<void*>(&interval), 
            sz
        );

        if (result < 0) throw system_error { errno, system_category(), "TCP Socket keep-alive probe interval"};
    }


    Time::Duration TCP_socket::keep_alive_interval() const
    {
        int interval    { };
        socklen_t sz    { static_cast<socklen_t>(sizeof(int)) };

        auto result = ::getsockopt(
            native_handle(),
            SOL_TCP, 
            TCP_KEEPINTVL,
            reinterpret_cast<void*>(&interval),
            &sz
        );

        if (result < 0) throw system_error { errno, system_category(), "TCP Socket keep-alive probe interval"};

        return Time::to_sec_duration(interval);
    }


    void TCP_socket::user_timeout(const Time::Duration& timeout_duration)
    {
        int timeout     { static_cast<int>(timeout_duration.milliseconds()) };
        socklen_t sz    { static_cast<socklen_t>(sizeof(int)) };

        auto result = ::setsockopt(
            native_handle(),
            SOL_TCP, 
            TCP_USER_TIMEOUT, 
            reinterpret_cast<void*>(&timeout), 
            sz
        );

        if (result < 0) throw system_error { errno, system_category(), "TCP Socket user timeout"};
    }


    Time::Duration TCP_socket::user_timeout() const
    {
        int timeout     { };
        socklen_t sz    { static_cast<socklen_t>(sizeof(int)) };

        auto result = ::getsockopt(
            native_handle(),
            SOL_TCP, 
            TCP_USER_TIMEOUT,
            reinterpret_cast<void*>(&timeout),
            &sz
        );

        if (result < 0) throw system_error { errno, system_category(), "TCP Socket user timeout"};

        return Time::to_msec_duration(timeout);
    }


    void TCP_socket::no_delay(bool enable)
    {
        int on          { enable ? 1 : 0 };
        socklen_t sz    { static_cast<socklen_t>(sizeof(int)) };

        auto result = ::setsockopt(
            native_handle(),
            SOL_TCP, 
            TCP_NODELAY, 
            reinterpret_cast<void*>(&on), 
            sz
        );

        if (result < 0) throw system_error { errno, system_category(), "TCP Socket TCP no-delay enable"};
    }

#elif _WIN32
    void TCP_socket::keep_alive(bool)                           { /* do nothing */ }
    bool TCP_socket::keep_alive() const                         { return { }; }

    void TCP_socket::keep_alive_idle(const Time::Duration&)     { /* do nothing */ }
    Time::Duration TCP_socket::keep_alive_idle() const          { return { }; }

    void TCP_socket::keep_alive_probes(unsigned)                { /* do nothing */ }
    unsigned TCP_socket::keep_alive_probes() const              { return { }; }

    void TCP_socket::keep_alive_interval(const Time::Duration&) { /* do nothing */ }
    Time::Duration TCP_socket::keep_alive_interval() const      { return { }; }

    void TCP_socket::user_timeout(const Time::Duration&)        { /* do nothing */ }
    Time::Duration TCP_socket::user_timeout() const             { return { }; }

    void TCP_socket::no_delay(bool)                             { /* do nothing */ } 

#endif

} // namespace Navtech::Networking