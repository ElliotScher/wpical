# WPIcal
![WPIcal](WPIcal.png)

WPIcal is a tool used to “calibrate” or empirically measure the position and orientation of the Apriltags on FRC fields. This tool was inspired by team 1538, The Holy Cows, who created a command-line tool to perform this field calibration: https://github.com/TheHolyCows/cowlibration-field. WPIcal aims to streamline field calibration by combining the needed camera calibration with the field calibration process into a user-friendly application.

## Overview
WPIcal measures the positions of the Apriltags on the field relative to each other. By selecting a tag to “pin” or use the default position for, WPIcal will create a field map, in which the other tags on the field will be moved to their measured positions relative to the pinned tag.

### Calibration Process
* Print out a copy of the ChArUco board used to calibrate your camera.

    ![ChArUco](ChArUco.png)
* Take a short video of the calibration board with your camera, make sure the board is on a flat, non-reflective surface.
* Take videos of the Apriltags you would like to calibrate.
* Ensure field calibration videos are in a separate directory on your machiine
* Run WPIcal application:
    * Select your camera calibration video.
    * Fill in the ChArUco board properties
    * Select the ideal Apriltag field map .json file provided by WPIlib.
    * Select the field video directory.
    * Select the pinned tag. It must be a tag that is in the calibration video
    * Select the processing FPS - Must be slower than the field calibration video FPS
    * Calibrate!
    * The output.json file will be generated in the field calibration video directory.
    * Upload your .json file to https://tools.limelightvision.io/map-builder to view your new field map!
    * You can now download your new field map and use it in your robot code or on your vision co-processors.

## Build
If you wish to build WPIcal from source or want to contribute, these steps will help you get started. This project uses CMake as its build tool, and vcpkg as its dependency manager.

### Windows
* Ensure you have [Visual Studio](https://visualstudio.microsoft.com/) installed with c++ desktop support.

    ![c++DesktopSupport](c++DesktopSupport.png)
* Run the following commands:
    ```
    git clone https://github.com/ElliotScher/wpical.git
    cd WPIcal
    git clone https://github.com/wpical.git
    cd vcpkg
    ./bootstrap-vcpkg.bat
    ./vcpkg integrate install
    ```
* To configure the cmake project, run:
    ```
    cmake -S <wpical_root_directory> -B <wpical_root_directory/build> -G "Visual Studio 17 2022"
    ```
* To build the cmake project, run: ```cmake --build build``` from inside the WPIcal root directory.

### Linux
* Clone the WPIcal git repository:
    ```
    git clone https://github.com/ElliotScher/wpical.git
    ```
* Run the following commands:
    ```
    cd WPIcal
    chmod +x setup.sh
    ./setup.sh
    ```
* To configure the cmake project, run:
    ```
    cmake -S <wpical_root_directory> -B <wpical_root_directory/build> -G "Ninja"
    ```
* To build the cmake project, run: ```cmake --build build``` from inside the WPIcal root directory.
