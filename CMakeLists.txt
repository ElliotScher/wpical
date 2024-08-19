cmake_minimum_required(VERSION 3.10)

set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")

project(WPIcal)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

include(cmake/modules/GenResources.cmake)
generate_resources(src/main/resources generated/main/cpp OV ov outlineviewer_resources_src)

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
find_package(GTest REQUIRED)

# Add the executable and specify the source files
add_executable(${PROJECT_NAME} 
    src/main/cpp/WPIcal.cpp 
    src/main/cpp/fieldcalibration.cpp 
    src/main/cpp/cameracalibration.cpp
    src/main/cpp/fieldmap.cpp
    src/main/cpp/fmap.cpp
    src/main/cpp/tagpose.cpp
    ${outlineviewer_resources_src}
    ${outlineviewer_rc}
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main/include
    ${CMAKE_CURRENT_SOURCE_DIR}/build/vcpkg_installed/x64-windows/include
    ${CMAKE_CURRENT_SOURCE_DIR}/mrcal-java/src
)

# Link the libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
    wpigui
    imgui
    mrcal_jni
    apriltag::apriltag
    Eigen3::Eigen
    Ceres::ceres
    nlohmann_json::nlohmann_json
    SuiteSparse::amd
    SuiteSparse::cholmod
    ${OpenCV_LIBS}
)

add_executable(
    tests
    src/main/cpp/fieldcalibration.cpp 
    src/main/cpp/cameracalibration.cpp
    src/main/cpp/fieldmap.cpp
    src/main/cpp/fmap.cpp
    src/main/cpp/tagpose.cpp
    src/test/test.cpp
)

target_include_directories(tests PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main/include
    ${CMAKE_CURRENT_SOURCE_DIR}/build/vcpkg_installed/x64-windows/include
    ${CMAKE_CURRENT_SOURCE_DIR}/mrcal-java/src
)

target_link_libraries(tests PRIVATE
    wpigui
    imgui
    mrcal_jni
    apriltag::apriltag
    Eigen3::Eigen
    Ceres::ceres
    nlohmann_json::nlohmann_json
    SuiteSparse::amd
    SuiteSparse::cholmod
    ${OpenCV_LIBS}
    GTest::GTest
)

if(WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE YES)
elseif(APPLE)
    set_target_properties(${PROJECT_NAME} PROPERTIES MACOSX_BUNDLE YES OUTPUT_NAME "WPIcal")
endif()

include(GoogleTest)
gtest_discover_tests(tests)

add_custom_command(TARGET tests
                   POST_BUILD
                   COMMAND ctest --output-on-failure
                   DEPENDS ${PROJECT_NAME})

add_dependencies(tests ${PROJECT_NAME})

target_compile_definitions(tests PRIVATE PROJECT_ROOT_PATH="${CMAKE_SOURCE_DIR}")