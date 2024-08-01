@echo off
@REM *** Simple CMake build script
@REM ***

@REM *** TODO - The build type on Windows must be Release
set BUILD_TYPE=Release

set PLATFORM=win64
set BUILD_ROOT=build
set SRC_ROOT=src
set BUILD_DIR=%BUILD_ROOT%\%PLATFORM%

cmake --build %BUILD_DIR% --config %BUILD_TYPE% -j