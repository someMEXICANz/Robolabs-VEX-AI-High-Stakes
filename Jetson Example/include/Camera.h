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


    int color_fps;
    int depth_fps;                                          
    
    void updateStreams();                                      
    void preprocessFrames(std::vector<float> &output); // Prepare & Process frame for Tensorrt engine
    std::shared_ptr<open3d::geometry::PointCloud> getPointCloud();
    int getColorFPS();
    int getDepthFPS();

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
  

    std::shared_ptr<rs2::depth_sensor> rs_sensor;
    rs2::stream_profile depth_stream;
    rs2::stream_profile color_stream;
    rs2::video_stream_profile depth_profile;
    rs2::video_stream_profile color_profile;

    rs2::frame color_frame; 
    rs2::frame depth_frame;                      
            
    rs2_intrinsics rs_intrinsic;
    open3d::camera::PinholeCameraIntrinsic o3d_intrinsic;
    Eigen::Matrix4d o3d_extrinsic;
    float depth_scale;    


    
    void updateLoop();
    bool initialize();
    bool reconnect();    

    bool running;
    bool connected;

    std::unique_ptr<std::thread> update_thread;
    std::mutex stream_mutex;
    
    cv::Mat convertFrameToMat(const rs2::frame& frame); // Convert RealSense frame to OpenCV Mat
    const std::chrono::milliseconds RECONNECT_DELAY{2500};



   
















};

#endif // CAMERA_H
