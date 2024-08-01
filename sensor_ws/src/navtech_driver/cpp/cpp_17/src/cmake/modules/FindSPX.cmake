if(PLATFORM STREQUAL "win64")
    set (SPX_LIB_VERSION "V1.89")
    set (SPX_LIB_ROOT "${CMAKE_SOURCE_DIR}/../lib/${PLATFORM}/SPx/${SPX_LIB_VERSION}")
    set (SPX_LIB_TYPE "win32/UniReleaseStaticMFC")

    if (NOT EXISTS ${SPX_LIB_ROOT})
        message(FATAL_ERROR "Could not locate SPx libraries at: ${SPX_LIB_ROOT}")
    endif ()

    set (SPX_INCLUDE_DIR ${SPX_LIB_ROOT}/Includes)

    set (SPXZLIB_LIBRARY ${SPX_LIB_ROOT}/Libs/${SPX_LIB_TYPE}/zlib-static_64.lib)
    set (SPX_LIBRARY     ${SPX_LIB_ROOT}/Libs/${SPX_LIB_TYPE}/SPxLibNoMFC_64.lib)
    set (SPXMHT_LIBRARY  ${SPX_LIB_ROOT}/Libs/${SPX_LIB_TYPE}/SPxLibMHT_64.lib)
    set (SPX127_LIBRARY  ${SPX_LIB_ROOT}/Libs/${SPX_LIB_TYPE}/SPxLib127_64.lib)
    set (MXST32_LIBRARY  ${SPX_LIB_ROOT}/Libs/${SPX_LIB_TYPE}/mxst32_64.lib)

elseif(PLATFORM STREQUAL "linux_x86_64")
    find_package(Threads REQUIRED)
    find_package(ZLIB REQUIRED)
    find_package(LibUSB REQUIRED)

    set (SPX_LIB_VERSION "V1.89")
    set (SPX_LIB_ROOT "${CMAKE_SOURCE_DIR}/../lib/${PLATFORM}/SPx/${SPX_LIB_VERSION}")
    set (SPX_LIB_TYPE "linux")

    message(STATUS "SPx libs for ${PLATFORM}: ${SPX_LIB_ROOT}")

    set (SPX_INCLUDE_DIR ${SPX_LIB_ROOT}/Includes)

    set(SPX_LIBRARY       ${SPX_LIB_ROOT}/Libs/${SPX_LIB_TYPE}/libspx_64.a)
    set(SPX127_LIBRARY    ${SPX_LIB_ROOT}/Libs/${SPX_LIB_TYPE}/libspx127_64.a)
    set(SPXMHT_LIBRARY    ${SPX_LIB_ROOT}/Libs/${SPX_LIB_TYPE}/libspxmht_64.a)
    set(SPXIRC_LIBRARY    ${SPX_LIB_ROOT}/Libs/${SPX_LIB_TYPE}/libirc.a)
    set(SPXLIN260_LIBRARY ${SPX_LIB_ROOT}/Libs/${SPX_LIB_TYPE}/libmxlin260_64.a)

elseif(PLATFORM STREQUAL "linux_arm")
    find_package(Threads REQUIRED)
    find_package(ZLIB REQUIRED)

    set (SPX_LIB_VERSION "V1.89")
    set (SPX_LIB_ROOT "${CMAKE_SOURCE_DIR}/../lib/${PLATFORM}/SPx/${SPX_LIB_VERSION}")
    set (SPX_LIB_TYPE "linux")

    set (SPX_INCLUDE_DIR ${SPX_LIB_ROOT}/Includes)

    set(SPX_LIBRARY     ${SPX_LIB_ROOT}/Libs/${SPX_LIB_TYPE}/libspx.a)
    set(SPX127_LIBRARY  ${SPX_LIB_ROOT}/Libs/${SPX_LIB_TYPE}/libspx127.a)
    set(SPXMHT_LIBRARY  ${SPX_LIB_ROOT}/Libs/${SPX_LIB_TYPE}/libspxmht.a)  
    set(SPXZLIB_LIBRARY ${SPX_LIB_ROOT}/Libs/${SPX_LIB_TYPE}/libz.a)  

elseif(PLATFORM STREQUAL "linux_aarch64")
    find_package(Threads REQUIRED)
    find_package(ZLIB REQUIRED)

    set (SPX_LIB_VERSION "V1.89")
    set (SPX_LIB_ROOT "${CMAKE_SOURCE_DIR}/../lib/${PLATFORM}/SPx/${SPX_LIB_VERSION}")
    set (SPX_LIB_TYPE "linux")

    set (SPX_INCLUDE_DIR ${SPX_LIB_ROOT}/Includes)

    set(SPX_LIBRARY     ${SPX_LIB_ROOT}/Libs/${SPX_LIB_TYPE}/libspx.a)
    set(SPX127_LIBRARY  ${SPX_LIB_ROOT}/Libs/${SPX_LIB_TYPE}/libspx127.a)
    set(SPXMHT_LIBRARY  ${SPX_LIB_ROOT}/Libs/${SPX_LIB_TYPE}/libspxmht.a)
    set(SPXZLIB_LIBRARY ${SPX_LIB_ROOT}/Libs/${SPX_LIB_TYPE}/libz.a)
endif()


if(SPX_INCLUDE_DIR AND SPX_LIBRARY AND SPXMHT_LIBRARY)
    set(SPX_FOUND TRUE)
    if(PLATFORM STREQUAL "linux_arm")
        set(SPX_LIBRARIES 
            ${SPX127_LIBRARY} 
            ${SPXMHT_LIBRARY} 
            ${SPX_LIBRARY} 
            ${SPXLIN260_LIBRARY} 
            ${SPXZLIB_LIBRARY}
        )
    
    elseif(PLATFORM STREQUAL "win64")
        set(SPX_LIBRARIES 
            ${SPX127_LIBRARY} 
            ${SPXMHT_LIBRARY} 
            ${SPX_LIBRARY} 
            ${SPXLIN260_LIBRARY} 
            ${SPXIRC_LIBRARY} 
            ${MXST32_LIBRARY} 
            ${SPXZLIB_LIBRARY}
        )
    
    elseif(PLATFORM STREQUAL "linux_aarch64")
        set(SPX_LIBRARIES 
            ${SPX127_LIBRARY} 
            ${SPXMHT_LIBRARY} 
            ${SPX_LIBRARY} 
            ${SPXLIN260_LIBRARY} 
            ${SPXZLIB_LIBRARY}
        )
    
    elseif(PLATFORM STREQUAL "linux_x86_64")
        set(SPX_LIBRARIES 
            ${SPX127_LIBRARY} 
            ${SPXMHT_LIBRARY} 
            ${SPX_LIBRARY} 
            ${SPXLIN260_LIBRARY} 
            ${SPXIRC_LIBRARY} 
            ${LibUSB_LIBRARIES} 
            ${ZLIB_LIBRARY}
        )
    endif()
    
    message(STATUS "SPx library version: ${SPX_LIB_VERSION}")
    message(STATUS "Found SPx includes:  ${SPX_INCLUDE_DIR}")
    message(STATUS "Found SPx Libraries: ${SPX_LIBRARIES}")
    
else(SPX_INCLUDE_DIR AND SPX_LIBRARY AND SPXMHT_LIBRARY)
    set(SPX_FOUND FALSE)
    
    if (SPX_FIND_REQUIRED)
        message(FATAL_ERROR "Could NOT find required package SPx")
    endif(SPX_FIND_REQUIRED)

endif(SPX_INCLUDE_DIR AND SPX_LIBRARY AND SPXMHT_LIBRARY)

mark_as_advanced(
    SPX_INCLUDE_DIR 
    SPX_LIBRARIES 
    SPX127_LIBRARY 
    SPXMHT_LIBRARY 
    SPX_LIBRARY 
    ZLIB_LIBRARY 
    SPXIRC_LIBRARY 
    SPXLIN260_LIBRARY
)
