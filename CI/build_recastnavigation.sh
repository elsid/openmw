#!/bin/sh -e

GENERATOR="${1}"

git clone https://github.com/elsid/recastnavigation.git
cd recastnavigation
git checkout 3686e06
mkdir build
cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DRECASTNAVIGATION_DEMO=OFF \
    -DRECASTNAVIGATION_TESTS=OFF \
    -DRECASTNAVIGATION_EXAMPLES=OFF \
    -DCMAKE_INSTALL_PREFIX=. \
    -G "${GENERATOR}" \
    ..
cmake --build .
cmake --build . --target install
