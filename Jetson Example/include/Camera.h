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
    bool getInferFrame(std::vector<float> &output); // Prepare & Process frame for Tensorrt engine
    std::shared_ptr<open3d::geometry::PointCloud> getPointCloud();
    std::shared_ptr<open3d::geometry::RGBDImage> getRGBDImage();
    
    

    

    // Thread Operations
    bool start();
    void stop();
    bool restart();

    // Status checks
    bool isConnected() const {return connected;}
    bool isRunning() const {return running;}
    bool isInitialized() const {return initialized;}

    void debug();


private:
    
    rs2::pipeline pipe;              // RealSense pipeline
    rs2::config config;              // Configuration for the pipeline
    rs2::align align_to;             // Align depth to color
    rs2::context context;
    rs2::device* device;
  
    rs2::frame color_frame; 
    rs2::frame depth_frame;  

    std::shared_ptr<open3d::geometry::Image> color_image;
    std::shared_ptr<open3d::geometry::Image> depth_image;                    
    
    void setIntrinsic(const rs2_intrinsics &intrinsics); 
    open3d::camera::PinholeCameraIntrinsic intrinsic;
    Eigen::Matrix4d extrinsic;

    int FPS;
    float depth_scale;

    // bool updateRGBDImage();
    std::shared_ptr<open3d::geometry::RGBDImage> current_RGBDImage;

    
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
