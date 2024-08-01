C++17 Implementation of the IA SDK
==================================

This folder contains a C++17 standard version of the IA SDK.

This version of the SDK was developed on Ubuntu 20.04. 
The Preferred compiler for bulding is Clang V10 but GCC 9.3.x should work.
The SDK has been tested with Visual Studio 17 2022 (MSVC 19.37.x).

To get started, use the following guides:
* `Building the SDK` gives instructions for setting up and compiling the SDK, either as a stand-alone project, or as part of your project.
* `Example projects` gives a brief overview of supplied applications and an overview of the supplied library utilities
* `Getting started` is a brief tutorial to show you how to program the SDK.

## Directory overview
```
.
├── build                           Build output directory.  Executables are located here.
├── doc                             Documentation
├── lib                             Third-party libraries
│   ├── linux_x86_64
│   └── win64
├── scripts                         Startup, configuration and build scripts
└── src                             SDK source code.
    ├── apps                        Example client/server applications
    ├── cmake                       CMake modules
    ├── navigation                  Navigation mode utilities
    ├── network                     Networking
    │   ├── core                    Base clients, Servers and networking utilities
    │   └── protocols               Communication protocols, and protocol-specific client/server types
    ├── protobuf                    Protocol buffers, for use with communication protocols
    └── utility                     Utility code.
        ├── architecture            General utilities to improve code structure    
        ├── containers              Container extensions
        ├── geometry                Coordinates, angles, etc.
        ├── math                    Mathematical types
        ├── protobuf                Protocol buffer helper utilities
        ├── radar_client            Utilities for use with radar clients
        ├── security                Security utilities
        ├── string                  Text processing and conversion tools
        ├── system                  Input and output handling
        ├── threading               Multi-tasking facilities
        ├── time                    Time and timing utilities
        └── units                   Physical units of measure
```
