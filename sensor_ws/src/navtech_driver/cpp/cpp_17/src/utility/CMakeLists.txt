cmake_minimum_required(VERSION 3.16)

project(sdk_utilities C CXX)

add_library(utility STATIC
    ${PROJECT_SOURCE_DIR}/geometry/Cartesian_coordinate.cpp
    ${PROJECT_SOURCE_DIR}/geometry/Euclidean_coordinate.cpp
    ${PROJECT_SOURCE_DIR}/geometry/Polar_coordinate.cpp
    ${PROJECT_SOURCE_DIR}/geometry/Spherical_coordinate.cpp

    ${PROJECT_SOURCE_DIR}/system/Log.cpp
    ${PROJECT_SOURCE_DIR}/system/Signal_handler.cpp
    ${PROJECT_SOURCE_DIR}/system/Option_parser.cpp

    ${PROJECT_SOURCE_DIR}/threading/Active.cpp
    ${PROJECT_SOURCE_DIR}/threading/Mutex.cpp

    ${PROJECT_SOURCE_DIR}/time/Time_utils.cpp
    ${PROJECT_SOURCE_DIR}/time/Timer.cpp

)

target_include_directories(utility INTERFACE
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/architecture
    ${PROJECT_SOURCE_DIR}/containers
    ${PROJECT_SOURCE_DIR}/geometry
    ${PROJECT_SOURCE_DIR}/math
    ${PROJECT_SOURCE_DIR}/protobuf
    ${PROJECT_SOURCE_DIR}/radar_client
    ${PROJECT_SOURCE_DIR}/security
    ${PROJECT_SOURCE_DIR}/string
    ${PROJECT_SOURCE_DIR}/system
    ${PROJECT_SOURCE_DIR}/threading
    ${PROJECT_SOURCE_DIR}/time
    ${PROJECT_SOURCE_DIR}/units
)

target_include_directories(utility PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/architecture
    ${PROJECT_SOURCE_DIR}/containers
    ${PROJECT_SOURCE_DIR}/geometry
    ${PROJECT_SOURCE_DIR}/math
    ${PROJECT_SOURCE_DIR}/protobuf
    ${PROJECT_SOURCE_DIR}/radar_client
    ${PROJECT_SOURCE_DIR}/security
    ${PROJECT_SOURCE_DIR}/string
    ${PROJECT_SOURCE_DIR}/system
    ${PROJECT_SOURCE_DIR}/threading
    ${PROJECT_SOURCE_DIR}/time
    ${PROJECT_SOURCE_DIR}/units
)

target_link_libraries(utility PRIVATE ${CMAKE_THREAD_LIBS_INIT})


