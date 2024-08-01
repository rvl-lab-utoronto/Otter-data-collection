cmake_minimum_required(VERSION 3.16)

project(TLS C CXX)

if (NOT PLATFORM STREQUAL "win64")
    add_library(tls STATIC
        ${PROJECT_SOURCE_DIR}/TLS_config.cpp
        ${PROJECT_SOURCE_DIR}/botan/Botan_certificate_file.cpp
        ${PROJECT_SOURCE_DIR}/botan/Botan_credentials_manager.cpp
        ${PROJECT_SOURCE_DIR}/botan/Botan_key_file.cpp
        ${PROJECT_SOURCE_DIR}/botan/Botan_policy.cpp
        ${PROJECT_SOURCE_DIR}/botan/Botan_server_credentials_manager.cpp
        ${PROJECT_SOURCE_DIR}/botan/Botan_TLS_server.cpp
        ${PROJECT_SOURCE_DIR}/botan/Botan_TLS_services.cpp
    )

    set(TLS_INC_DIRS
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/botan
        ${PROJECT_SOURCE_DIR}/null
    )

    target_include_directories(tls INTERFACE
        ${TLS_INC_DIRS}
    )

    target_include_directories(tls PRIVATE
        ${TLS_INC_DIRS}
    )

    target_link_libraries(tls
        utility
        networking
        protobuf
        ${BOTAN2_LIBRARY}
    )

else ()
    add_library(tls STATIC
        ${PROJECT_SOURCE_DIR}/TLS_config.cpp
    )

    target_include_directories(tls INTERFACE
        ${PROJECT_SOURCE_DIR}
    )

    target_include_directories(tls PRIVATE
        ${PROJECT_SOURCE_DIR}
    )

endif ()

