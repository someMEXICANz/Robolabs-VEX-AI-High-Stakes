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

// struct WallFeature {
//     Eigen::Vector3d normal;
//     double distance;
//     Eigen::Vector3d centroid;
//     double confidence;
//     std::chrono::system_clock::time_point timestamp;
// };

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

    // access methods
    int getPPS();
    cv::Mat getOccupancyMap() const;
    
private:
    
    Camera& camera;
    // RobotPosition& robot_position;

    void updateLoop();
    std::unique_ptr<std::thread> update_thread;
    mutable std::mutex data_mutex;
    bool running;

    int PPS;
    
    // // Map properties
    float map_width;
    float map_height;
    float map_resolution;
   
    // Data Storage
    std::shared_ptr<open3d::geometry::PointCloud> legacy_point_cloud;
    open3d::t::geometry::PointCloud tensor_point_cloud;


    std::tuple<Eigen::Vector4d, open3d::core::Tensor> ground_plane;
    cv::Mat occupancy_map;
    
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


    open3d::t::geometry::RGBDImage current_image;
    open3d::t::geometry::RGBDImage prevoius_image;
    
    void setIntrinsicAndExtrinsic();
    open3d::core::Tensor intrinsic_tensor;
    open3d::core::Tensor extrinsic_tensor;
    bool extrin_intrin_init;


    
    
    // Processing methods
    bool processPointCloud();
    bool processRGBDImage();
    bool segmentPlanes();
    void computeOdometry();
    void refinePosition();
    void ClusterObstacles();
    void updateOccupancyMap();

};

#endif // FIELD_MAPPER_H