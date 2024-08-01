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
#include <sstream>
#include <iomanip>
#include <algorithm>

#include "Colossus_UDP_network_message.h"
#include "net_conversion.h"


// Constants
//
constexpr unsigned int largest_valid_message { 8192 };
constexpr unsigned int largest_payload       { 8184 };
constexpr std::uint8_t version               { 2 };

namespace Navtech::Networking::Colossus_protocol::UDP {

    Message::Message()
    {
        initialize();
    }


    Message::Message(Type msg_type)
    {
        initialize();
        type(msg_type);
    }


    Message::Message(const IP_address& ip_addr, Message::ID id) :
        address     { ip_addr },
        identity    { id }
    {
        initialize();
    }


    Message::Message(const Message::Buffer& message)
    {
        replace(message);
    }


    Message::Message(Message::Buffer&& message)
    {
        replace(std::move(message));
    }


    Message::Message(Const_iterator message_start, std::size_t message_sz)
    {
        replace(message_start, message_sz);
    }

    
    Message::Message(const IP_address& ip_addr, Message::ID id, const Message::Buffer& message) :
        address         { ip_addr },
        identity        { id }
    {
        replace(message);
    }


    Message::Message(const IP_address& ip_addr, Message::ID id, Message::Buffer&& message) :
        address         { ip_addr },
        identity        { id }
    {
        replace(std::move(message));
    }


    Message::Message(const IP_address& ip_addr, ID id, Const_iterator message_start, std::size_t message_sz) :
        address         { ip_addr },
        identity        { id }
    {
        replace(message_start, message_sz);
    }
    

    Message::ID Message::id() const
    {
        return identity;
    }


    void Message::id(Message::ID new_id)
    {
        identity = new_id;
    }


    Type Message::type() const
    {
        auto header = Header::overlay_onto(data.data());
        return header->type;
    }


    void Message::type(Type t)
    {
        auto header = Header::overlay_onto(data.data());
        header->type = t;
    }


    const IP_address& Message::ip_address() const
    {
        return address;
    }
    

    void Message::ip_address(const IP_address& ip_addr)
    {
        address = ip_addr;
    }


    void Message::radar_serial(std::uint16_t serial_num)
    {
        auto header = Header::overlay_onto(data.data());
        header->radar_serial = to_uint16_network(serial_num);
    }


    std::uint16_t Message::radar_serial() const
    {
        auto header = Header::overlay_onto(data.data());
        return to_uint16_host(header->radar_serial);
    }


    bool Message::is_valid() const
    {
        if (data.empty())                                           return false;
        if (!is_version_valid())                                    return false;
        if (static_cast<unsigned>(type()) > largest_valid_message)  return false;
        if (payload_size() >= largest_payload)                      return false;
        
        return true;
    }


    std::size_t Message::size() const
    {
        return data.size();
    }


    std::size_t Message::payload_size() const
    {
        if (data.empty()) return 0;

        auto header = Header::overlay_onto(data.data());
        return to_uint32_host(header->payload_size);
    }


    void Message::update_payload_size()
    {
        auto header = Header::overlay_onto(data.data());
        header->payload_size = to_uint32_network(static_cast<std::uint32_t>(data.size() - header_size()));
    }


    Message::Iterator Message::payload_begin()
    {
        return data.data() + header_size();
    }


    Message::Const_iterator Message::payload_begin() const
    {
        return data.data() + header_size();
    }


    Message::Iterator Message::payload_end()
    {
        return &(*data.end());
    }


    Message::Const_iterator Message::payload_end() const
    {
        return &(*data.cend());
    }


    std::string Message::to_string() const
    {
        using namespace std;

        ostringstream os { };

        // Print out the header, plus a marker to show there
        // is more data (the payload) in the message.
        //
        for (std::size_t i { 0 }; i < header_size(); ++i) {
            os << setw(2) << setfill('0') << hex << static_cast<int>(data[i]) << " ";
        }

        if (data.size() > header_size()) {
            os << "...";
        }

        return os.str();
    }
    

    void  Message::replace(const std::vector<std::uint8_t>& src)
    {
        data = src;
    }


    void Message::replace(std::vector<std::uint8_t>&& src)
    {
        using std::move;

        data = move(src);
    }
        
    
    void Message::replace(Message::Const_iterator src_start, std::size_t src_size)
    {
        using std::copy_n;
        using std::back_inserter;

        data.clear();
        data.reserve(src_size);
        copy_n(src_start, src_size, back_inserter(data));
    }


    std::vector<std::uint8_t> Message::relinquish()
    {
        using std::move;
        using std::vector;
        using std::uint8_t;

        // add_signature();
        return vector<uint8_t> { move(data) };
    }


    Message::Iterator Message::begin()
    {
        return &(*data.begin());
    }


    Message::Const_iterator Message::begin() const
    {
        return &(*data.begin());
    }


    Message::Iterator Message::end()
    {
        return &(*data.end());
    }


    Message::Const_iterator Message::end() const
    {
        return &(*data.end());
    }


    Buffer_view Message::data_view() const
    {
        return Buffer_view { data };
    }

    
    Message& Message::append(const Message::Buffer& protocol_buffer)
    {
        using std::begin;
        using std::end;
        using std::copy;
        using std::back_inserter;

        // data.reserve(data.size() + protocol_buffer.size());
        // copy(begin(protocol_buffer), end(protocol_buffer), back_inserter(data));

        auto protobuf_offset = data.size();

        data.resize(data.size() + protocol_buffer.size());
        std::memcpy(
            data.data() + protobuf_offset,
            protocol_buffer.data(),
            protocol_buffer.size()
        );
       
        update_payload_size();
        has_protobuf = true;
        return *this;
    }


    Message& Message::append(Message::Buffer&& protocol_buffer)
    {
        using std::begin;
        using std::end;
        using std::copy;
        using std::back_inserter;

        // Take ownership to ensure correct move semantics
        // for client code
        //
        Buffer temp { move(protocol_buffer) };  

        // data.reserve(data.size() + temp.size());
        // copy(begin(temp), end(temp), back_inserter(data));

        auto protobuf_offset = data.size();

        data.resize(data.size() + temp.size());
        std::memcpy(
            data.data() + protobuf_offset,
            temp.data(),
            temp.size()
        );

        update_payload_size();
        has_protobuf = true;
        return *this;
    }


    Message& Message::append(const std::string& protocol_buffer)
    {
        using std::begin;
        using std::end;
        using std::copy;
        using std::back_inserter;

        data.reserve(data.size() + protocol_buffer.size());
        copy(begin(protocol_buffer), end(protocol_buffer), back_inserter(data));
       
        update_payload_size();
        has_protobuf = true;
        return *this;
    }



    Message& Message::append(std::string&& protocol_buffer)
    {
        using std::begin;
        using std::end;
        using std::copy;
        using std::back_inserter;

        // Take ownership to ensure correct move semantics
        // for client code
        //
        std::string temp { move(protocol_buffer) };  

        data.reserve(data.size() + temp.size());
        copy(begin(temp), end(temp), back_inserter(data));

        update_payload_size();
        has_protobuf = true;
        return *this;
    }


    Message& Message::operator<<(const Message::Buffer& protocol_buffer)
    {
        return append(protocol_buffer);
    }


    Message& Message::operator<<(Message::Buffer&& protocol_buffer)
    {
        return append(std::move(protocol_buffer));
    }


    Message& Message::operator<<(const std::string& protocol_buffer)
    {
        return append(protocol_buffer);
    }


    Message& Message::operator<<(std::string&& protocol_buffer)
    {
        return append(std::move(protocol_buffer));
    }


    void Message::initialize()
    {
        data.resize(header_size());
        add_version(version);
    }


    bool Message::is_version_valid() const
    {
        auto header = Header::overlay_onto(data.data());

        return (header->version == version);
    }


    void Message::add_version(std::uint8_t version)
    {
        auto header = Header::overlay_onto(data.data());

        header->version = version;
    }

} // namespace Navtech::Networking::Colossus_protocol::UDP