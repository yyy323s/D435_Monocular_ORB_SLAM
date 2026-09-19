cmake_minimum_required(VERSION 3.16)

project(D435_Monocular_ORB_SLAM VERSION 1.0.0 LANGUAGES C CXX)

# CMake 3.30+ removed FindBoost. Prefer Boost's package configuration while
# keeping this project usable with older CMake releases.
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(BUILD_D435_APP "Build the live Intel RealSense D435 application" ON)

# The workstation keeps Pangolin in a versioned user-local prefix. Users on
# other machines can still pass -DPangolin_DIR=... in the usual CMake way.
if(DEFINED ENV{HOME})
    list(PREPEND CMAKE_PREFIX_PATH "$ENV{HOME}/.local/pangolin/0.9.5")
endif()

find_package(OpenCV 4.4 REQUIRED COMPONENTS
    core imgproc highgui features2d calib3d imgcodecs videoio)
find_package(Eigen3 3.1 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)
find_package(Boost CONFIG REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)

if(TARGET pango_display)
    set(ORB_SLAM3_PANGOLIN_TARGET pango_display)
else()
    set(ORB_SLAM3_PANGOLIN_TARGET ${Pangolin_LIBRARIES})
endif()

# Build the exact DBoW2 and g2o variants shipped with ORB-SLAM3 so their ABI
# always matches the core library.
add_subdirectory(Thirdparty/DBoW2 EXCLUDE_FROM_ALL)
add_subdirectory(Thirdparty/g2o EXCLUDE_FROM_ALL)

set(ORB_SLAM3_SOURCES
    src/System.cc
    src/Tracking.cc
    src/LocalMapping.cc
    src/LoopClosing.cc
    src/ORBextractor.cc
    src/ORBmatcher.cc
    src/FrameDrawer.cc
    src/Converter.cc
    src/MapPoint.cc
    src/KeyFrame.cc
    src/Atlas.cc
    src/Map.cc
    src/MapDrawer.cc
    src/Optimizer.cc
    src/Frame.cc
    src/KeyFrameDatabase.cc
    src/Sim3Solver.cc
    src/Viewer.cc
    src/ImuTypes.cc
    src/G2oTypes.cc
    src/CameraModels/Pinhole.cpp
    src/CameraModels/KannalaBrandt8.cpp
    src/OptimizableTypes.cpp
    src/MLPnPsolver.cpp
    src/GeometricTools.cc
    src/TwoViewReconstruction.cc
    src/Config.cc
    src/Settings.cc
)

add_library(ORB_SLAM3 SHARED ${ORB_SLAM3_SOURCES})
add_library(ORB_SLAM3::ORB_SLAM3 ALIAS ORB_SLAM3)

target_include_directories(ORB_SLAM3
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/CameraModels
        ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Sophus
        ${CMAKE_BINARY_DIR}/Thirdparty/g2o
)

target_compile_definitions(ORB_SLAM3 PUBLIC COMPILEDWITHC11)
target_compile_options(ORB_SLAM3 PRIVATE -Wall -Wextra -Wpedantic)

target_link_libraries(ORB_SLAM3
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
        ${ORB_SLAM3_PANGOLIN_TARGET}
    PRIVATE
        DBoW2
        g2o
        Boost::serialization
        OpenSSL::Crypto
        Threads::Threads
)

set_target_properties(ORB_SLAM3 PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

set(ORB_VOCABULARY_ARCHIVE
    "${CMAKE_CURRENT_SOURCE_DIR}/Vocabulary/ORBvoc.txt.tar.gz")
set(ORB_VOCABULARY_FILE
    "${CMAKE_BINARY_DIR}/Vocabulary/ORBvoc.txt")

if(NOT EXISTS "${ORB_VOCABULARY_FILE}")
    if(NOT EXISTS "${ORB_VOCABULARY_ARCHIVE}")
        message(FATAL_ERROR "Missing ${ORB_VOCABULARY_ARCHIVE}")
    endif()
    message(STATUS "Extracting ORB vocabulary (one-time operation)")
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Vocabulary")
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E tar xzf "${ORB_VOCABULARY_ARCHIVE}"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/Vocabulary"
        RESULT_VARIABLE ORB_VOCABULARY_EXTRACT_RESULT
    )
    if(NOT ORB_VOCABULARY_EXTRACT_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to extract ORB vocabulary")
    endif()
endif()

if(BUILD_D435_APP)
    find_package(realsense2 2.50 REQUIRED)

    add_executable(mono_d435 apps/mono_d435.cpp)
    target_link_libraries(mono_d435 PRIVATE ORB_SLAM3::ORB_SLAM3)

    if(TARGET realsense2::realsense2)
        target_link_libraries(mono_d435 PRIVATE realsense2::realsense2)
    else()
        target_include_directories(mono_d435 PRIVATE ${realsense2_INCLUDE_DIR})
        target_link_libraries(mono_d435 PRIVATE ${realsense2_LIBRARY})
    endif()

    target_compile_options(mono_d435 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_definitions(mono_d435 PRIVATE
        DEFAULT_VOCABULARY_PATH="${ORB_VOCABULARY_FILE}"
        DEFAULT_SETTINGS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/D435_Monocular.yaml"
        DEFAULT_RUNTIME_DIRECTORY="${CMAKE_BINARY_DIR}/runtime"
    )
    set_target_properties(mono_d435 PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )
endif()
