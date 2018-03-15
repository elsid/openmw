#!/bin/sh -e

export CXX=clang++
export CC=clang

CI/build_recastnavigation.sh 'Unix Makefiles'
RECASTNAVIGATION_DIR="$(pwd)/recastnavigation/build"

DEPENDENCIES_ROOT="/private/tmp/openmw-deps/openmw-deps"
QT_PATH=`brew --prefix $macos_qt_formula`
mkdir build
cd build

cmake \
-D CMAKE_PREFIX_PATH="$DEPENDENCIES_ROOT;$QT_PATH" \
-D CMAKE_OSX_DEPLOYMENT_TARGET="10.9" \
-D CMAKE_OSX_SYSROOT="macosx10.12" \
-D CMAKE_BUILD_TYPE=Release \
-D OPENMW_OSX_DEPLOYMENT=TRUE \
-D DESIRED_QT_VERSION=5 \
-D BUILD_ESMTOOL=FALSE \
-D BUILD_MYGUI_PLUGIN=FALSE \
-D DETOUR_ROOT="${RECASTNAVIGATION_DIR}" \
-D RECAST_ROOT="${RECASTNAVIGATION_DIR}" \
-G"Unix Makefiles" \
..
