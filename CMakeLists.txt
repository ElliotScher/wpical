cmake_minimum_required(VERSION 3.10)

set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")

project(WPIcal)

set(OSBitness 64)
if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(OSBitness 32)
endif()

set(FullOutputDir "${CMAKE_SOURCE_DIR}/bin/${CMAKE_SYSTEM_NAME}${OSBitness}/${CMAKE_BUILD_TYPE}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${FullOutputDir}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${FullOutputDir}")

find_package(OpenCV CONFIG REQUIRED)
find_package(apriltag CONFIG REQUIRED)
find_package(ceres CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

add_executable(
    ${PROJECT_NAME}
    src/cpp/main.cpp
    src/cpp/fieldcalibration.cpp
    src/cpp/cameracalibration.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/src/include
    ${CMAKE_SOURCE_DIR}/wpigui/src/include
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_video
    opencv_videoio
    Ceres::ceres
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    apriltag::apriltag
)