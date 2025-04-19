#ifndef FIELD_MAPPER_H
#define FIELD_MAPPER_H

#include <thread>
#include <mutex>
#include <iostream>
#include <vector>
#include <map>
#include <open3d/Open3D.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "Camera.h"
#include "RobotPosition.h"


class FieldMapper {
public:
    
    explicit FieldMapper(Camera& Camera); //, RobotPosition& Position); // Constructor 
    ~FieldMapper();                                                     // Destructor

    // Delete copy constructor and assignment operator
    FieldMapper(const FieldMapper&) = delete;
    FieldMapper& operator=(const FieldMapper&) = delete;

   
    bool start();
    void stop();
    bool restart();

    bool isRunning() const {return running;}
    bool isInitialized() const {return initialized;}

    void setExtrinsic(float roll, float pitch, float yaw, float x, float y, float z);

    // access methods
    int getPPS();
    cv::Mat getOccupancyMap() const;
    
private:
    
    Camera& camera;
    // RobotPosition& robot_position;

    void updateLoop();
    bool initialize();

    std::unique_ptr<std::thread> update_thread;
    mutable std::mutex data_mutex;
    bool running;
    bool initialized;

    int PPS;
    
    // // Map properties
    float map_width;
    float map_height;
    float map_resolution;

    //DBSCAN clustering parameters
    float cluster_eps;
    int cluster_min_points;
    
    // Plane segmentation parameters
    float plane_distance_threshold;
    int plane_ransac_n;
    int plane_num_iterations;
    float ground_threshold;

    // Filtering parameters
    float min_height_threshold;
    float max_height_threshold;
   
    // Data Storage
    std::shared_ptr<open3d::geometry::PointCloud> legacy_point_cloud;
    open3d::t::geometry::PointCloud tensor_point_cloud;

    std::tuple<Eigen::Vector4d, open3d::core::Tensor> ground_plane;
    cv::Mat occupancy_map;
    
    open3d::t::geometry::RGBDImage current_image;
    open3d::t::geometry::RGBDImage prevoius_image;

    float depth_scale;
    open3d::core::Tensor intrinsic_tensor;
    open3d::core::Tensor extrinsic_tensor;

    open3d::core::Tensor current_transform;
    open3d::core::Tensor prevoius_transform;


    

    
    // Processing methods
    bool processPointCloud();
    bool processRGBDImage();
    bool segmentPlanes();
    bool computeOdometry();
    void refinePosition();
    void ClusterObstacles();
    void updateOccupancyMap();

};

#endif // FIELD_MAPPER_H