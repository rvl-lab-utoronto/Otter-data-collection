cmake_minimum_required(VERSION 3.16)

project(ia_sdk C CXX)
enable_testing()


# ----------------------------------------------------------------------------------------------------------------------
#
if (PLATFORM STREQUAL "win64")
    message(STATUS "Building for Windows")

    # Protobuf libraries
    # 
    set(PROTOBUF_VERSION "v3.19.4")
    set(PROTOBUF_PATH ${PROJECT_SOURCE_DIR}/../lib/${PLATFORM}/Protobuf/${PROTOBUF_VERSION})
    set(Protobuf_INCLUDE_DIR ${PROTOBUF_PATH}/include)
    set(Protobuf_PROTOC_EXECUTABLE ${PROTOBUF_PATH}/bin/protoc.exe)
    set(PROTOBUF_LIBRARY ${PROTOBUF_PATH}/lib/libprotobuf.lib)

    # ASIO libraries
    # 
    set(ASIO_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../lib/${PLATFORM}/Asio/include)
    include_directories(${ASIO_INCLUDE_DIR})

    message(STATUS "ASIO include path: ${ASIO_INCLUDE_DIR}")
    
    # Compiler options
    # 
    if (MSVC)
        message(STATUS "Configuring for MSVC")
    
        set (BUILD_SHARED_LIBS FALSE)
        set_property(GLOBAL PROPERTY USE_FOLDERS ON)    
        
        add_compile_options(
            /std:c++17
            /W3
            /MT
        )

        add_compile_definitions(
            NOMINMAX
            _CRT_SECURE_NO_WARNINGS
            _UNICODE
            UNICODE
            WIN32_LEAN_AND_MEAN
        )

        set (CMAKE_MFC_FLAG 0)

    endif (MSVC)

else ()
    message(STATUS "Building for Linux (ARM/host)")

    add_compile_options(
        -std=c++17
        -Wall
        -Wfatal-errors
        # -Wextra
        # -Werror - Turned off, because of SPx libraries
        -pthread
        -Wl,--no-as-needed 
        -fPIC 
        -Wdouble-promotion 
        -ftree-vectorize 
        -ffast-math 
        -funsafe-math-optimizations
        $<$<CONFIG:DEBUG>:-ggdb>
        $<$<CONFIG:DEBUG>:-O0>
        $<$<CONFIG:RELEASE>:-O3>
        $<$<STREQUAL:"${PLATFORM}","linux_aarch64">:-march=armv8-a>
        $<$<STREQUAL:"${PLATFORM}","linux_aarch64">:-mtune=cortex-a53>
        # $<$<STREQUAL:"${PLATFORM}","linux_aarch64">:-D__aarch64__>
        $<$<STREQUAL:"${PLATFORM}","linux_arm">:-march=armv7-a>
        $<$<STREQUAL:"${PLATFORM}","linux_arm">:-mtune=cortex-a9>
        $<$<STREQUAL:"${PLATFORM}","linux_arm">:-mfpu=neon>
        $<$<STREQUAL:"${PLATFORM}","linux_arm">:-mfloat-abi=hard>
        $<$<STREQUAL:"${PLATFORM}","linux_arm">:-Wno-psabi>
        # $<$<STREQUAL:"${PLATFORM}","linux_arm">:-D__arm>
    )

    add_compile_definitions(
        SPX_X11_SUPPORTED=0
        $<$<STREQUAL:"${PLATFORM}","linux_aarch64">:__aarch64__>
        $<$<STREQUAL:"${PLATFORM}","linux_arm">:__arm>
    )


endif ()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

# ----------------------------------------------------------------------------------------------------------------------
# Library dependencies
# 
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})

find_package(Botan2 REQUIRED)
include_directories(${BOTAN2_INCLUDE_DIR})

# find_package(SPX REQUIRED)
# include_directories(${SPX_INCLUDE_DIR})
# link_directories(${SPX_LIBRARIES})

# ----------------------------------------------------------------------------------------------------------------------
# Top-level include path
# 
include_directories(${PROJECT_SOURCE_DIR})

# ----------------------------------------------------------------------------------------------------------------------
# Sub-libraries
#
add_subdirectory(protobuf)
add_subdirectory(network)
add_subdirectory(utility)
add_subdirectory(navigation)

# ----------------------------------------------------------------------------------------------------------------------
# Executables
#
add_subdirectory(apps)

