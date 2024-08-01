cmake_minimum_required(VERSION 3.16)

project(sdk_apps C CXX)

# Output all applications to the same folder
#
set(APP_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

# ----------------------------------------------------------------------------------------------------------------------
# Colossus client
# 
add_executable(colossus_client
    ${PROJECT_SOURCE_DIR}/Colossus_client_main.cpp
)

set_target_properties(colossus_client
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${APP_OUTPUT_PATH}
)

target_link_libraries(colossus_client
    protobuf
    utility
    networking
)


# ----------------------------------------------------------------------------------------------------------------------
# Point-cloud client
# 
add_executable(pointcloud_client
    ${PROJECT_SOURCE_DIR}/pointcloud_client_main.cpp
)

set_target_properties(pointcloud_client
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${APP_OUTPUT_PATH}
)

target_link_libraries(pointcloud_client
    protobuf
    utility
    networking
)


# ----------------------------------------------------------------------------------------------------------------------
# Navigation-mode client
#
add_executable(navigation_client
    ${PROJECT_SOURCE_DIR}/navigation_main.cpp
)

set_target_properties(navigation_client
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${APP_OUTPUT_PATH}
)

target_link_libraries(navigation_client
    protobuf
    utility
    networking
    navigation
)


# ----------------------------------------------------------------------------------------------------------------------
# NMEA client
#
add_executable(nmea_client
    ${PROJECT_SOURCE_DIR}/NMEA_client_main.cpp
)

set_target_properties(nmea_client
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${APP_OUTPUT_PATH}
)

target_link_libraries(nmea_client
    protobuf
    utility
    networking
)


# ----------------------------------------------------------------------------------------------------------------------
# NMEA server

add_executable(nmea_server
    ${PROJECT_SOURCE_DIR}/NMEA_server_main.cpp
)

set_target_properties(nmea_server
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${APP_OUTPUT_PATH}
)

target_link_libraries(nmea_server
    protobuf
    utility
    networking
)


# ----------------------------------------------------------------------------------------------------------------------
# Point-cloud writer
#
add_executable(pointcloud_writer
    ${PROJECT_SOURCE_DIR}/pointcloud_writer_main.cpp
)

set_target_properties(pointcloud_writer
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${APP_OUTPUT_PATH}
)

target_link_libraries(pointcloud_writer
    protobuf
    utility
    networking
)


# ----------------------------------------------------------------------------------------------------------------------
# TCP relay
#
add_executable(tcp_relay
    ${PROJECT_SOURCE_DIR}/tcp_relay_main.cpp
)

set_target_properties(tcp_relay
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${APP_OUTPUT_PATH}
)

target_link_libraries(tcp_relay
    protobuf
    utility
    networking
)

# ----------------------------------------------------------------------------------------------------------------------
# CAT-240 client
#
add_executable(cat240_client
    ${PROJECT_SOURCE_DIR}/Cat240_client_main.cpp
)

set_target_properties(cat240_client
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${APP_OUTPUT_PATH}
)

target_link_libraries(cat240_client
    protobuf
    utility
    networking
)

# ----------------------------------------------------------------------------------------------------------------------
# Point cloud target writer
#
add_executable(pointcloud_target_writer
    ${PROJECT_SOURCE_DIR}/Pointcloud_target_writer_main.cpp
)

set_target_properties(pointcloud_target_writer
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${APP_OUTPUT_PATH}
)

target_link_libraries(pointcloud_target_writer
    protobuf
    utility
    networking
    navigation
)

# ----------------------------------------------------------------------------------------------------------------------
# 3D Point cloud writer
#
add_executable(pointcloud_3d_writer
    ${PROJECT_SOURCE_DIR}/Pointcloud_3d_writer_main.cpp
)

set_target_properties(pointcloud_3d_writer
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${APP_OUTPUT_PATH}
)

target_link_libraries(pointcloud_3d_writer
    protobuf
    utility
    networking
    navigation
)