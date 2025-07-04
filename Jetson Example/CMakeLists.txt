cmake_minimum_required(VERSION 3.19)
project(CameraTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 
set(CMAKE_POLICY_DEFAULT_CM)

cmake_policy(SET CMP0167 NEW)
cmake_policy(SET CMP0072 NEW)

# Find CUDA
find_package(CUDAToolkit REQUIRED)
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

# Find OpenCV
set(WITH_CUDA ON)
find_package(OpenCV REQUIRED)

# Find Open3D
find_package(Open3D REQUIRED)

# Find Eigen
find_package(Eigen3 REQUIRED)
add_definitions(-DEIGEN_USE_THREADS) 

find_package(Boost REQUIRED COMPONENTS system thread)

#Find RealSense
find_package(PkgConfig REQUIRED)
pkg_check_modules(REALSENSE2 REQUIRED realsense2)

include_directories(CameraTest PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/SH2
    ${OpenCV_INCLUDE_DIRS}
    ${Open3D_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CUDAToolkit_INCLUDE_DIRS}
)

add_executable(CameraTest
    src/main.cpp
    src/Camera.cpp
    src/Model.cpp
    src/ObjectDetection.cpp
    src/GPS.cpp
    # src/Position.cpp
    # src/RobotPosition.cpp
    src/BrainCommStats.cpp
    src/BrainComm.cpp
    src/I2C_Device.cpp
    # src/IMU.cpp
    src/WS_UPS.cpp
    # src/FieldMapper.cpp
    src/BNO085.cpp
    src/SH2/shtp.c
    src/SH2/sh2.c
    src/SH2/sh2_util.c
    src/SH2/sh2_SensorValue.c
)

target_compile_options(CameraTest PRIVATE -Wno-deprecated-declarations)

target_compile_features(CameraTest PRIVATE cxx_std_17)

target_link_libraries(CameraTest PRIVATE
    ${OpenCV_LIBS}
    ${LIBSERIAL_LIBRARIES}
    ${Boost_LIBRARIES}  
    Open3D::Open3D
    cudart
    nvinfer
    nvonnxparser
    rt
    pthread
)