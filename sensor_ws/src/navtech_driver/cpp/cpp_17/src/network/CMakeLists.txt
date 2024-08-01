cmake_minimum_required(VERSION 3.16)

project(Network C CXX)

add_subdirectory(tls)

add_library(networking STATIC
    ${PROJECT_SOURCE_DIR}/core/sockets/Socket.cpp
    ${PROJECT_SOURCE_DIR}/core/sockets/TCP_socket.cpp
    ${PROJECT_SOURCE_DIR}/core/sockets/UDP_socket.cpp
    ${PROJECT_SOURCE_DIR}/core/sdk/sdk.cpp
    
    # Colossus TCP
    # 
    ${PROJECT_SOURCE_DIR}/protocols/Colossus/TCP/messages/Colossus_TCP_network_message.cpp
    ${PROJECT_SOURCE_DIR}/protocols/Colossus/TCP/client/Colossus_TCP_client.cpp
    ${PROJECT_SOURCE_DIR}/protocols/Colossus/TCP/event_handling/Colossus_TCP_events.cpp
    ${PROJECT_SOURCE_DIR}/protocols/Colossus/TCP/server/Colossus_TCP_server.cpp

    # Colossus UDP
    # 
    ${PROJECT_SOURCE_DIR}/protocols/Colossus/UDP/messages/Colossus_UDP_network_message.cpp
    ${PROJECT_SOURCE_DIR}/protocols/Colossus/UDP/client/Colossus_UDP_client.cpp
    ${PROJECT_SOURCE_DIR}/protocols/Colossus/UDP/event_handling/Colossus_UDP_events.cpp
    ${PROJECT_SOURCE_DIR}/protocols/Colossus/UDP/server/Colossus_UDP_server.cpp
    
    # CP
    # 
    ${PROJECT_SOURCE_DIR}/protocols/CP/messages/CP_network_message.cpp
    ${PROJECT_SOURCE_DIR}/protocols/CP/event_handling/CP_events.cpp
    
    # CAT-240
    # 
    ${PROJECT_SOURCE_DIR}/protocols/CAT_240/messages/Cat240_message.cpp
    ${PROJECT_SOURCE_DIR}/protocols/CAT_240/client/Cat240_client.cpp
    ${PROJECT_SOURCE_DIR}/protocols/CAT_240/event_handling/Cat240_events.cpp
    ${PROJECT_SOURCE_DIR}/protocols/CAT_240/server/Cat240_server.cpp

    # NMEA
    # 
    ${PROJECT_SOURCE_DIR}/protocols/NMEA/messages/NMEA_message.cpp
    ${PROJECT_SOURCE_DIR}/protocols/NMEA/messages/NMEA_parser.cpp
    ${PROJECT_SOURCE_DIR}/protocols/NMEA/client/NMEA_client.cpp
    ${PROJECT_SOURCE_DIR}/protocols/NMEA/event_handling/NMEA_events.cpp
    ${PROJECT_SOURCE_DIR}/protocols/NMEA/server/NMEA_server.cpp

    ${PROJECT_SOURCE_DIR}/utilities/IP_address.cpp
    ${PROJECT_SOURCE_DIR}/utilities/MAC_address.cpp
    ${PROJECT_SOURCE_DIR}/utilities/net_conversion.cpp
)

set(NETWORK_INC_DIRS
    ${PROJECT_SOURCE_DIR}/utilities

    ${PROJECT_SOURCE_DIR}/core/sdk
    ${PROJECT_SOURCE_DIR}/core/clients
    ${PROJECT_SOURCE_DIR}/core/connections
    ${PROJECT_SOURCE_DIR}/core/connections/secure
    ${PROJECT_SOURCE_DIR}/core/connections/datagram
    ${PROJECT_SOURCE_DIR}/core/connections/bytestream
    ${PROJECT_SOURCE_DIR}/core/connections/messaging
    ${PROJECT_SOURCE_DIR}/core/event_handling
    ${PROJECT_SOURCE_DIR}/core/servers
    ${PROJECT_SOURCE_DIR}/core/sockets

    ${PROJECT_SOURCE_DIR}/protocols/

    ${PROJECT_SOURCE_DIR}/protocols/CAT_240
    ${PROJECT_SOURCE_DIR}/protocols/CAT_240/client
    ${PROJECT_SOURCE_DIR}/protocols/CAT_240/event_handling
    ${PROJECT_SOURCE_DIR}/protocols/CAT_240/messages
    ${PROJECT_SOURCE_DIR}/protocols/CAT_240/server
    
    ${PROJECT_SOURCE_DIR}/protocols/Colossus
    
    ${PROJECT_SOURCE_DIR}/protocols/Colossus/TCP
    ${PROJECT_SOURCE_DIR}/protocols/Colossus/TCP/client
    ${PROJECT_SOURCE_DIR}/protocols/Colossus/TCP/event_handling
    ${PROJECT_SOURCE_DIR}/protocols/Colossus/TCP/messages
    ${PROJECT_SOURCE_DIR}/protocols/Colossus/TCP/server
    
    ${PROJECT_SOURCE_DIR}/protocols/Colossus/UDP
    ${PROJECT_SOURCE_DIR}/protocols/Colossus/UDP/client
    ${PROJECT_SOURCE_DIR}/protocols/Colossus/UDP/event_handling
    ${PROJECT_SOURCE_DIR}/protocols/Colossus/UDP/messages
    ${PROJECT_SOURCE_DIR}/protocols/Colossus/UDP/server
    
    ${PROJECT_SOURCE_DIR}/protocols/CP
    ${PROJECT_SOURCE_DIR}/protocols/CP/client
    ${PROJECT_SOURCE_DIR}/protocols/CP/event_handling
    ${PROJECT_SOURCE_DIR}/protocols/CP/messages
    ${PROJECT_SOURCE_DIR}/protocols/CP/server
    
    ${PROJECT_SOURCE_DIR}/protocols/NMEA
    ${PROJECT_SOURCE_DIR}/protocols/NMEA/client
    ${PROJECT_SOURCE_DIR}/protocols/NMEA/event_handling
    ${PROJECT_SOURCE_DIR}/protocols/NMEA/messages
    ${PROJECT_SOURCE_DIR}/protocols/NMEA/server
)

target_include_directories(networking INTERFACE
    ${NETWORK_INC_DIRS}
)

target_include_directories(networking PRIVATE
    ${NETWORK_INC_DIRS}
)

target_link_libraries(networking
    utility
    protobuf
    tls
)

