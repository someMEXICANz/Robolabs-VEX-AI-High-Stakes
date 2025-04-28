cmake_minimum_required(VERSION 3.19)
project(CameraTest)

# Set C++ standard (still keeping this for compatibility, but will also use target_compile_features)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # This is important for VSCode

# Find CUDA
find_package(CUDAToolkit REQUIRED)
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

# Find Boost
find_package(Boost REQUIRED COMPONENTS thread system)

# Find the i2c library - consolidated to a single search
find_library(I2C_LIBRARY 
    NAMES i2c
    PATHS /usr/lib/aarch64-linux-gnu
)
if(NOT I2C_LIBRARY)
    message(FATAL_ERROR "libi2c not found, please install 'libi2c-dev' package")
endif()

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find Open3D
find_package(Open3D REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)
add_definitions(-DEIGEB_USE_THREADS)

# Find RealSense
find_package(PkgConfig REQUIRED)
pkg_check_modules(REALSENSE2 REQUIRED realsense2)

# Find Eigen
find_package(Eigen3 REQUIRED)
add_definitions(-DEIGEN_USE_THREADS) # Keeping add_definitions for this macro

# Add executable (Including GPS files)
add_executable(CameraTest
    src/main.cpp
    src/Camera.cpp
    src/Model.cpp
    src/ObjectDetection.cpp
    src/GPS.cpp
    src/Position.cpp
    src/RobotPosition.cpp
    src/BrainCommStats.cpp
    src/BrainComm.cpp
    src/IMU.cpp
    src/UPS.cpp
    src/FieldMapper.cpp
    src/RobotVisualizer.cpp
)

# Modern approach: target-specific include directories
target_include_directories(CameraTest PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/external/kalman/include
    ${OpenCV_INCLUDE_DIRS}
    ${REALSENSE2_INCLUDE_DIRS}
    ${Open3D_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CUDAToolkit_INCLUDE_DIRS}
)

# Modern approach: target-specific link directories
target_link_directories(CameraTest PRIVATE
    ${OpenCV_LIBRARY_DIRS}
    ${REALSENSE2_LIBRARY_DIRS}
)

# Modern approach: target-specific compile options
target_compile_options(CameraTest PRIVATE
    -Wno-deprecated-declarations
)

# Modern approach: target-specific compile features
target_compile_features(CameraTest PRIVATE cxx_std_17)

# Link libraries
target_link_libraries(CameraTest PRIVATE
    ${OpenCV_LIBS}
    ${REALSENSE2_LIBRARIES}
    ${LIBSERIAL_LIBRARIES}
    ${Boost_LIBRARIES}  
    ${I2C_LIBRARY}
    Open3D::Open3D
    cudart
    nvinfer
    nvonnxparser
    rt
    pthread
)