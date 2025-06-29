cmake_minimum_required(VERSION 3.19)
project(CameraTest)

# Set C++ standard (still keeping this for compatibility, but will also use target_compile_features)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # This is important for VSCode

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find Open3D
find_package(Open3D REQUIRED)

# Find Eigen
find_package(Eigen3 REQUIRED)
add_definitions(-DEIGEN_USE_THREADS) 

# Find CUDA
find_package(CUDAToolkit REQUIRED)
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

find_package(Boost REQUIRED COMPONENTS system thread)

#Find RealSense
find_package(PkgConfig REQUIRED)
pkg_check_modules(REALSENSE2 REQUIRED realsense2)


include_directories(CameraTest PRIVATE
    ${CMAKE_SOURCE_DIR}/include
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
    src/Position.cpp
    src/RobotPosition.cpp
    src/BrainCommStats.cpp
    src/BrainComm.cpp
    src/IMU.cpp
    src/UPS.cpp
    src/FieldMapper.cpp
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
    ${LIBSERIAL_LIBRARIES}
    ${Boost_LIBRARIES}  
    Open3D::Open3D
    cudart
    nvinfer
    nvonnxparser
    rt
    pthread
)