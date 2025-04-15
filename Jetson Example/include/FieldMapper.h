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
    // Constructor and Destructor
    explicit FieldMapper(Camera& Camera); //, RobotPosition& Position);
    ~FieldMapper();

    // Delete copy constructor and assignment operator
    FieldMapper(const FieldMapper&) = delete;
    FieldMapper& operator=(const FieldMapper&) = delete;

   
    bool start();
    void stop();
    bool restart();

    // Map access methods
    cv::Mat getOccupancyMap() const;
    
   

    

private:
    // References to external components
    Camera& camera;
    // RobotPosition& robot_position;

    void updateLoop();
    std::unique_ptr<std::thread> update_thread;
    mutable std::mutex data_mutex;
    bool running;
    
    // // Map properties
    float map_width;
    float map_height;
    float map_resolution;
   
    // Data Storage
    open3d::geometry::PointCloud legacy_point_cloud;
    open3d::geometry::PointCloud tensor_point_cloud;


    std::tuple<Eigen::Vector4d, open3d::core::Tensor> ground_plane;
    cv::Mat occupancy_map;
    
    // DBSCAN clustering parameters
    float cluster_eps;
    int cluster_min_points;
    
    // Plane segmentation parameters
    float plane_distance_threshold;
    int plane_ransac_n;
    int plane_num_iterations;

    // // Filtering parameters
    float min_height_threshold;
    float max_height_threshold;

    const float ground_threshold; // cos(~18 degrees)
    
    // Processing methods
    bool processPointCloud();
    bool segmentPlanes();
    void refinePosition();
    void ClusterObstacles();
    void updateOccupancyMap();

};

#endif // FIELD_MAPPER_H