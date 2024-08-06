cmake_minimum_required(VERSION 3.10)

set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")

project(WPIcal)

include(cmake/modules/GenResources.cmake)
generate_resources(src/resources generated/main/cpp OV ov outlineviewer_resources_src)

if(WIN32)
    set(outlineviewer_rc src/win/outlineviewer.rc)
elseif(APPLE)
    set(MACOSX_BUNDLE_ICON_FILE ov.icns)
    set(APP_ICON_MACOSX src/mac/ov.icns)
    set_source_files_properties(${APP_ICON_MACOSX} PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")
endif()

# Add include directories for the project
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/include)

# Add subdirectories for external libraries
add_subdirectory(wpigui)
add_subdirectory(imgui)
add_subdirectory(mrcal-java)

# Find required packages
find_package(apriltag REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(suitesparse REQUIRED)
find_package(GTSAM REQUIRED)

# Add the executable and specify the source files
add_executable(${PROJECT_NAME} 
    src/cpp/cameracalibration.cpp
    src/cpp/fieldcalibration.cpp 
    src/cpp/fieldmap.cpp
    src/cpp/fmap.cpp
    src/cpp/gtsam_utils.cpp
    src/cpp/tagmodel.cpp
    src/cpp/tagpose.cpp
    src/cpp/WPIcal.cpp
    ${outlineviewer_resources_src}
    ${outlineviewer_rc}
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/build/vcpkg_installed/x64-windows/include
    ${CMAKE_CURRENT_SOURCE_DIR}/mrcal-java/src
)

# Link the libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
    wpigui
    imgui
    mrcal_jni
    gtsam
    apriltag::apriltag
    Eigen3::Eigen
    Ceres::ceres
    nlohmann_json::nlohmann_json
    SuiteSparse::amd
    SuiteSparse::cholmod
    ${OpenCV_LIBS}
)

if(WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE YES)
elseif(APPLE)
    set_target_properties(${PROJECT_NAME} PROPERTIES MACOSX_BUNDLE YES OUTPUT_NAME "WPIcal")
endif()