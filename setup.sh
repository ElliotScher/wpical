#! /bin/bash

#!/bin/bash

# Function to install a package
install_package() {
    local package_name=$1

    if command -v apt-get &> /dev/null; then
        echo "Installing $package_name..."
        sudo apt-get update && sudo apt-get install -y "$package_name"
    elif command -v dnf &> /dev/null; then
        echo "Installing $package_name..."
        sudo dnf install -y "$package_name"
    elif command -v yum &> /dev/null; then
        echo "Installing $package_name..."
        sudo yum install -y "$package_name"
    elif command -v pacman &> /dev/null; then
        echo "Installing $package_name..."
        sudo pacman -Sy --noconfirm "$package_name"
    elif command -v zypper &> /dev/null; then
        echo "Installing $package_name..."
        sudo zypper install -y "$package_name"
    elif command -v emerge &> /dev/null; then
        echo "Installing $package_name..."
        sudo emerge "$package_name"
    else
        echo "No supported package manager detected. Please install $package_name manually."
        exit 1
    fi
}

install_package curl 
install_package git 
install_package zip
install_package unzip
install_package tar
install_package cmake
install_package ninja-build
install_package build-essential
install_package libxinerama-dev
install_package libxcursor-dev
install_package xorg-dev
install_package libglu1-mesa-dev
install_package pkg-config
install_package bison
install_package meson
install_package libx11-dev
install_package libxft-dev
install_package libxext-dev
install_package autoconf
install_package automake
install_package autoconf-archive
install_package libtool
install_package libxi-dev
install_package libxtst-dev
install_package nasm
install_package linux-libc-dev
install_package libxml2
install_package libxml2-dev
install_package gfortran
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
./vcpkg integrate install
cd ..
sudo cmake -DCMAKE_BUILD_TYPE=Release
sudo cmake -S . -B build -G "Ninja" -DVCPKG_ROOT="./vcpkg"