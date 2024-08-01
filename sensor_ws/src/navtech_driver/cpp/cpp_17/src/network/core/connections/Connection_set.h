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
#ifndef CONNECTION_SET_H
#define CONNECTION_SET_H

#include <vector>
#include <string>
#include <sstream>
#include <cstddef>
#include <mutex>
#include <tuple>

#include "Connection_manager_traits.h"
#include "IP_address.h"

namespace Navtech::Networking {

    template <Protocol protocol, Transport transport, TLS::Type tls>
    class Connection_set {
    public:
        // Type aliases.
        // The '_Ty' postfix denotes a template type. Rather than being
        // supplied as template parameters on the class (which would be unwieldy) 
        // these parameters are looked up from the Stream_server_traits class, 
        // using the appropriate combination of protocol, transport and TLS type
        //
        using Connection_mgr_traits = Connection_manager_traits<protocol, transport, tls>;
        using Connection_Ty         = typename Connection_mgr_traits::Connection;
        using Connection_ptr_Ty     = typename Connection_mgr_traits::Connection_ptr;
        using Message_Ty            = typename Connection_Ty::Message_Ty;
        using ID_Ty                 = typename Connection_mgr_traits::ID;
        using Container_Ty          = std::vector<Connection_ptr_Ty>;
    
        std::pair<bool, ID_Ty> exists(const IP_address& ip_addr);
        IP_address address_of(ID_Ty id) const;

        void add(Connection_ptr_Ty&& connection);
        bool replace(ID_Ty id, Connection_ptr_Ty&& connection);
        bool remove(ID_Ty id);
        void remove_all();

        const Connection_Ty& operator[](ID_Ty id) const;

        template <typename T> 
        void send(T&& msg);

        std::size_t size() const;
        std::string to_string() const;
    
    private:
        Container_Ty connections { };
        mutable std::mutex   mtx { };

        void send_to_all(const Message_Ty& msg);
        
        template <typename T> 
        void send_to_id(T&& msg);
    };


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection_set<protocol, transport, tls>::add(Connection_set<protocol, transport, tls>::Connection_ptr_Ty&& connection)
    {
        std::lock_guard lock { mtx };
        connections.push_back(std::move(connection));
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    bool Connection_set<protocol, transport, tls>::remove(ID_Ty id)
    {
        std::lock_guard lock { mtx };

        auto itr = std::remove_if(
            connections.begin(),
            connections.end(),
            [id](const auto& c) { return c->id() == id; }
        );

        bool removed { itr != connections.end() };

        if (removed) connections.erase(itr, connections.end());
    
        return removed;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection_set<protocol, transport, tls>::remove_all()
    {
        std::lock_guard lock { mtx };
        
        connections.clear();
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    std::pair<bool, typename Connection_set<protocol, transport, tls>::ID_Ty> 
    Connection_set<protocol, transport, tls>::exists(const IP_address& ip_addr)
    {
        std::lock_guard lock { mtx };

        auto itr = std::find_if(
            connections.begin(),
            connections.end(),
            [&ip_addr](const auto& c) { return c->remote_endpoint().ip_address == ip_addr; }
        );

        bool found { itr != connections.end() };

        if (found) return std::make_pair(found, (*itr)->id());
        else       return { };            
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    IP_address Connection_set<protocol, transport, tls>::address_of(Connection_set<protocol, transport, tls>::ID_Ty id) const
    {
        IP_address addr { IP_address::null() };

        std::lock_guard lock { mtx };

        auto itr = std::find_if(
            connections.begin(),
            connections.end(),
            [&id](const auto& c) { return c->id() == id; }
        );

        if (itr != connections.end()) {
            addr = (*itr)->remote_endpoint().ip_address;
        }

        return addr;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    bool Connection_set<protocol, transport, tls>::replace(
        Connection_set<protocol, transport, tls>::ID_Ty id,
        Connection_set<protocol, transport, tls>::Connection_ptr_Ty&& connection
    )
    {
        std::lock_guard lock { mtx };

        bool replaced { };
        
        auto itr = std::remove_if(
            connections.begin(),
            connections.end(),
            [&id](const auto& c) { return c->id() == id; }
        );

        if (itr != connections.end()) {
            *itr = std::move(connection);
            replaced = true;
        }
       
        return replaced;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    const typename Connection_set<protocol, transport, tls>::Connection_Ty& 
    Connection_set<protocol, transport, tls>::operator[](ID_Ty id) const
    {
        std::lock_guard lock { mtx };

        auto itr = std::find_if(
            connections.begin(),
            connections.end(),
            [&id](const auto& c) { return c->id() == id; }
        );

        return *(*itr);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    template <typename T>
    void Connection_set<protocol, transport, tls>::send(T&& msg)
    {
        std::lock_guard lock { mtx };

        // ID zero is the 'broadcast' address
        //
        if (msg.id() == 0) send_to_all(msg);                    // Always copy!
        else               send_to_id(std::forward<T>(msg));    // move/copy, as req'd.
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Connection_set<protocol, transport, tls>::send_to_all(const Message_Ty& msg)
    {
        // NOTE - Do not lock!
        // This function is called under the lock acquired in send()
        
        for (auto& conx : connections) conx->send(msg);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    template <typename T>
    void Connection_set<protocol, transport, tls>::send_to_id(T&& msg)
    {
        // NOTE - Do not lock!
        // This function is called under the lock acquired in send()

        auto itr = std::find_if(
            connections.begin(),
            connections.end(),
            [&msg](const auto& c) { return c->id() == msg.id(); }
        );

        if (itr != connections.end()) (*itr)->send(std::forward<T>(msg));
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    std::size_t Connection_set<protocol, transport, tls>::size() const
    {
        std::lock_guard lock { mtx };
        return connections.size();
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    std::string Connection_set<protocol, transport, tls>::to_string() const
    {
        std::stringstream stream { };
        std::lock_guard lock { mtx };

        if (connections.size() == 0) return "[0]";

        stream << "[";

        for (auto itr = connections.begin(); itr != connections.end(); itr++) {
            if (itr != connections.begin()) stream << ", ";
            stream << (*itr)->id();
            if (!(*itr)->is_enabled()) stream << "*";
        }

        stream << "]";

        return stream.str();      
    }


} // namespace Navtech::Networking

#endif // CONNECTION_SET_H