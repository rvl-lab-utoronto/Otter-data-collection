@echo off
@REM *** Simple CMake launcher
@REM ***

set PLATFORM=win64
set BUILD_ROOT=build
set SRC_ROOT=src
set BUILD_DIR=%BUILD_ROOT%\%PLATFORM%

@REM *** Get the build type.  Note this is case-insensitive
@REM ***

if [%1] == [] goto invalid_build

if /I %1 == Release goto build_release
if /I %1 == Debug   goto build_debug

:invalid_build
echo Invalid build parameters: bootstrap_win64.bat %1 %2
echo You must specify a build type. For example:
echo   .\bootstrap_win64.bat Release
echo or:
echo   .\bootstrap_win64.bat Debug
echo.
exit 

:build_debug
set BUILD_TYPE=Debug
goto run_build

:build_release
set BUILD_TYPE=Release
goto run_build

:run_build
echo Running CMake for %PLATFORM%, build type: %BUILD_TYPE%
echo.

:build
@REM *** Create the build directory
@REM ***
echo Creating the %BUILD_DIR% build directory...

rmdir /S /Q %BUILD_DIR%\
mkdir %BUILD_DIR%

@REM cmake -G "Visual Studio 17 2022" -A x64 -T v141 -DCMAKE_BUILD_TYPE=%BUILD_TYPE% -DPLATFORM=%PLATFORM% -S source -B %BUILD_DIR%

cmake -DCMAKE_BUILD_TYPE=%BUILD_TYPE% -DPLATFORM=%PLATFORM% -S %SRC_ROOT% -B %BUILD_DIR%

@REM *** Finished
@REM ***
echo.
echo Done.