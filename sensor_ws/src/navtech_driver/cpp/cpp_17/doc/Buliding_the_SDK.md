Building the SDK
================

This document gives basic instructions for building the SDK.  It covers both Linux and Windows.

## Prerequisites
### CMake
The SDK is designed to be built using the CMake tools.  These must be installed onto your build platform.

CMake download and installation instructions can be found [here](https://cmake.org/download/).


### Compiler
The SDK is build using C++17.  Your compiler must support this version of the language as the bare minimum.


### Linux
The SDK requires the linux build essentials and the Google protocol buffer libraries to compile.
For more information on installing protocol buffers, see [Google Protocol buffers](https://github.com/protocolbuffers/protobuf/)

```shell
sudo apt install build-essential clang g++ protobuf-compiler libprotobuf-dev cmake
```

### Windows
The SDK has been tested with Visual Studio 2017; and _should_ work with any later version.
The SDK has not been tested with MinGW.

Building the SDK using CMake allows it to be built from the command-line (which is what these instructions do). CMake on Windows will generate a `.sln` file that may be opened with Visual Studio if required.  This facility is not the recommended one, and has not been tested.

For more information on getting started with Visual Studio, go [here](https://code.visualstudio.com/learn/get-started/basics)


## Building
### Linux
This assumes that the SDK has been cloned into the folder `~/iasdk`.

To simplify the (arcane) CMake syntax, a simple batch script has been written to automate the build configuration process.  The script creates a folder, `build`, which holds the build artefacts and output.

```shell
scripts/linux/bootstrap.sh [ debug | release ]
```
Unless otherwise specified, the default build type is `Debug`.

To build the SDK CMake syntax may be used, for example:
```shell
cmake --build build/linux/Debug --config Debug -j
```

A simple script, `build.sh` has also been provided, to avoid the need for memorizing CMake syntax:
```shell
scripts/linux/build.sh [ debug | release ]
```

The output executables are placed in thr directory `build/<platform>/<build-type>/bin`, below the build-type folder.  For example:
```
/iasdk/build/linux/Debug/bin/colossus_client
/iasdk/build/linux/Debug/bin/navigation_client
/iasdk/build/linux/Debug/bin/connection_tester
/iasdk/build/linux/Debug/bin/cat240_client
/iasdk/build/linux/Debug/bin/nmea_server
/iasdk/build/linux/Debug/bin/nmea_client
```

### Windows
**NOTE: AT PRESENT, ONLY THE RELEASE CONFIGURATION IS SUPPORTED ON WINDOWS**

This assumes that the SDK has been cloned into the folder `iasdk`.

To simplify the (arcane) CMake syntax, a simple batch script has been written to automate the build configuration process.  The script creates a folder, `build`, which holds the build artifacts and output.

```shell
scripts\win64\bootstrap.bat [ debug | release ]
```
Unless otherwise specified, the default build type is `Debug`.

To build the SDK CMake syntax may be used, for example:
```shell
cmake --build build\win64\ --config Release -j
```

Alternatively, CMake will generate an ia_sdk.sln file in `.\build\win64`.
This file can be open in Visual Studio, which can then be used to build the SDK using the Build/Rebuild Solution menu items.

A simple script, `build.bat` has also been provided, to avoid the need for memorizing CMake syntax:
```shell
scripts\win64\build.bat [ debug | release ]
```

The output executables are placed in thr directory `build\<platform>\bin\<build-type>`, below the build-type folder.  For example:
```
iasdk\build\win64\bin\Release/colossus_client
iasdk\build\win64\bin\Release/navigation_client
iasdk\build\win64\bin\Release/connection_tester
iasdk\build\win64\bin\Release/cat240_client
iasdk\build\win64\bin\Release/nmea_server
iasdk\build\win64\bin\Release/nmea_client
```


## Using the VS Code build task
The `.vscode` folder contains a build task configuration for invoking CMake.
To build:
* hit ***ctrl-shift-b***
* select ***Build***

The following options are available:
```
Make        Compile the debug configuration of the project (all executables)
Clean       Clean the project (next build forces a re-compile of all files)
Configure   Re-run CMake on the project; no build
Build       Configure, then make the debug configuration (same as Configure + Make)
```

## Using Visual Studio
Running the CMake configuration (via `scripts\win64\bootstrap.bat`) will generate a Visual Studio `.sln` file, in
`\build\win64`.

The `.sln` can be opened and built in Visual Studio.
 
------------------------------------------------------------------------------------------------------------------------
## Using the iasdk for your own project/out of source build instructions
In order to incorporate the iasdk into your own project, you must edit your CMake file to perform an out of source build of the iasdk. This can be achieved via the following method:


#### Clone the SDK repository
Clone the IA SDK repository.  We recommend you clone it alongside the rest of your project code.


### Update the CMakeLists.txt for your project
The following will set the root src folder of the SDK. This path is relative to your current project's root directory, and must be changed accordingly.

```cmake
# Define the SDK path.
set(SDK_PATH ${PROJECT_SOURCE_DIR}/../../iasdk/cpp/cpp_17/src)
```

If using protocol buffers ("protobuf") in your project (for example parsing the protobuf messages received from a radar) then include the following.

```cmake
# Find and include directories
set (Protobuf_USE_STATIC_LIBS ON)
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})
```

Add the iasdk subdirectories to your project.  Note you have to specify where the SDK build objects will be located. Usually you will locate the SDK build objects in the same location as your other build artifacts.

```cmake
# Add libraries
add_subdirectory(${SDK_PATH}/utility    ${CMAKE_CURRENT_BINARY_DIR}/utility)
add_subdirectory(${SDK_PATH}/protobuf   ${CMAKE_CURRENT_BINARY_DIR}/protobuf)
add_subdirectory(${SDK_PATH}/network    ${CMAKE_CURRENT_BINARY_DIR}/network)
add_subdirectory(${SDK_PATH}/navigation ${CMAKE_CURRENT_BINARY_DIR}/navigation)
```

Link your project to the iasdk libraries.

```cmake
target_link_libraries(<your_project_name>  
    utility  
    protobuf 
    networking 
    navigation
)
```

## Example
Below is a complete CMakeLists.txt file for a project which uses the iasdk and performs an out of source build.

```cmake
# Define CMake minimum required version
cmake_minimum_required(VERSION 3.16)

# Define project name
#
project(<my_project_name>)

# Define the SDK path.  This assumes the SDK is located
# alongside your other source code.  Modify this path to
# match where you have place the SDK repository.
#
set(SDK_PATH ${PROJECT_SOURCE_DIR}/../iasdk/cpp/cpp_17/src)

# Add compile options.
# The SDK requires C++17 and -pthread to build.
# Other compile options may be omitted.
#
add_compile_options(
    -std=c++17
    -pthread
    
    # Other compiler options, as required...
)

# Find and include directories
#
set (Protobuf_USE_STATIC_LIBS ON)
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})

# Add libraries
#
add_subdirectory(${SDK_PATH}/utility    ${CMAKE_CURRENT_BINARY_DIR}/utility)
add_subdirectory(${SDK_PATH}/protobuf   ${CMAKE_CURRENT_BINARY_DIR}/protobuf)
add_subdirectory(${SDK_PATH}/network    ${CMAKE_CURRENT_BINARY_DIR}/network)
add_subdirectory(${SDK_PATH}/navigation ${CMAKE_CURRENT_BINARY_DIR}/navigation)

add_executable(<my_exectuable_name> <my_executable_source.cpp>) 

# Place the output executable in a known location
#
set_target_properties(<my_exectuable_name> 
    PROPERTIES 
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin 
) 
 
target_link_libraries(<my_exectuable_name>  
    utility  
    protobuf 
    networking 
    navigation 
) 
```