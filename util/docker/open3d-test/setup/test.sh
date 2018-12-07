#!/bin/sh

echo

git clone https://github.com/IntelVCL/Open3D.git open3d
cd open3d
git submodule update --init --recursive
echo

echo building STATIC...
mkdir -p build
cd build
cmake .. -DPYTHON_EXECUTABLE=/usr/bin/${PYTHON} \
         -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_UNIT_TESTS=ON
echo
make -j
echo

echo running the unit tests...
./bin/unitTests

echo cleaning...
cd ..
rm -rf build
echo

echo building SHARED...
mkdir -p build
cd build
cmake .. -DBUILD_SHARED_LIBS=ON \
         -DPYTHON_EXECUTABLE=/usr/bin/${PYTHON} \
         -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_UNIT_TESTS=ON
echo
make -j
echo

echo running the unit tests...
./bin/unitTests
