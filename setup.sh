#! /bin/bash

sudo apt-get update && sudo apt-get upgrade
sudo apt-get install curl git zip unzip tar cmake ninja-build build-essential libxinerama-dev libxcursor-dev xorg-dev libglu1-mesa-dev pkg-config bison meson libx11-dev libxft-dev libxext-dev autoconf automake autoconf-archive libtool libxi-dev libxtst-dev nasm linux-libc-dev libxml2 libxml2-dev
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
./vcpkg integrate install
cd ..
sudo cmake -DCMAKE_BUILD_TYPE=Release
sudo cmake -S . -B ./build -G "Ninja" -DCMAKE_BUILD_TYPE=Release -Wno-dev