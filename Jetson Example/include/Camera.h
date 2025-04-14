#ifndef CAMERA_H
#define CAMERA_H


#include <stdexcept>
#include <mutex>
#include <thread>
#include <iostream>
#include <vector>

#include "open3d/Open3D.h"
#include <librealsense2/rs.hpp>
#include <librealsense2/rs.h>
#include <opencv2/opencv.hpp>
#include <ObjectDetection.h>
#include <open3d/t/geometry/RGBDImage.h>
#include <open3d/t/geometry/PointCloud.h>
#include <Eigen/Dense>
#include <librealsense2/h/rs_types.h>




class Camera {


public:
    explicit Camera();                                                   // Constructor
    ~Camera();                                                  // Destructor

    // Delete copy constructor and assignment operator
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    void setExtrinsic(float roll, float pitch, float yaw, 
                      float x,    float y,     float z);     

    int getFPS();
    void getInferFrame(std::vector<float> &output); // Prepare & Process frame for Tensorrt engine
    open3d::t::geometry::PointCloud getPointCloud();

    

    // Thread Operations
    bool start();
    void stop();
    bool restart();

    // Status checks
    bool isConnected() const {return connected;}
    bool isRunning() const {return running;}


private:
    
    rs2::pipeline pipe;              // RealSense pipeline
    rs2::config config;              // Configuration for the pipeline
    rs2::align align_to;             // Align depth to color
    rs2::context context;
    rs2::device* device;
  
    rs2::frame color_frame; 
    rs2::frame depth_frame;                      
    
    void setIntrinsic(const rs2_intrinsics &intrinsics); 
    open3d::core::Tensor intrinsic;
    open3d::core::Tensor extrinsic;

    int FPS;
    float depth_scale;

    
    void updateLoop();   
    bool findDevice();
    void initialize();
    bool connect();    

    bool running;
    bool connected;
    bool initialized;

    const int frame_width;
    const int frame_height;

    std::unique_ptr<std::thread> update_thread;
    std::mutex stream_mutex;

    const std::chrono::milliseconds RECONNECT_DELAY{2500};



   
















};

#endif // CAMERA_H
