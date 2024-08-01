# ----------------------------------------------------------------------------------------
# A simple script to remove the need to remember arcane CMake syntax.
# This script will build either the hosted or target variations of the Tracker.
# The following default configurations are provided.  These match the standard CMake
# configurations:

# $ ./build.sh        Build linux host debug configuration
# $ ./build.sh arm    Build target release configuration
# $ ./build.sh linux  Build host debug configuration.

#!/bin/sh

set -o errexit
set -o nounset
USAGE="Usage: $(basename $0) [ arm | linux] [ debug | release ] [ clean ]"

BUILD_ROOT=./build
TARGET=linux
CONFIG=Debug
CLEAN=

for arg; do
  case "$arg" in
    --help|-h)      echo $USAGE; exit 0;;
    arm)            TARGET=arm;   CONFIG=Release;;  
    linux)          TARGET=linux; CONFIG=Debug;;
    debug)          CONFIG=Debug;;
    release)        CONFIG=Release;;
    clean)          CLEAN=1  ;;
    *)              echo -e "unknown option $arg\n$USAGE" >&2;  exit 1 ;;
  esac
done

BUILD_DIR=${BUILD_ROOT}/${TARGET}/${CONFIG}

echo "Building ${CONFIG} configuration for ${TARGET}"

if [[ ! -d ${BUILD_DIR} ]]; then
    echo "Can't find ${BUILD_DIR}"
    exit 1
fi

[[ -n $CLEAN ]] && cmake --build $BUILD_DIR --target clean

cmake --build $BUILD_DIR --config ${CONFIG} -j