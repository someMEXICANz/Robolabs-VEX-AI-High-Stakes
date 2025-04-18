#include <FieldMapper.h>


using namespace open3d;

FieldMapper::FieldMapper(Camera& Camera) // , RobotPosition& Position) 
    : camera(Camera),
      // robot_position(Position),
      running(false),
      PPS(0),
      map_width(3.66f),
      map_height(3.66f),
      map_resolution(0.05f), 
      cluster_eps(0.2f),
      cluster_min_points(10),
      plane_distance_threshold(0.05f),
      plane_ransac_n(3),
      plane_num_iterations(1000),
      ground_threshold(.95),
      min_height_threshold(0.1f),
      max_height_threshold(1.5f),
      extrin_intrin_init(false)
{
    start();
}

// Destructor implementation
FieldMapper::~FieldMapper() {
    stop();
}

bool FieldMapper::start() 
{
    if (running) 
    {
        std::cerr << "Field Mapper update thread is already running" << std::endl; 
        return true;
    }
    
    try{
        running = true;
        update_thread = std::make_unique<std::thread>(&FieldMapper::updateLoop, this);
        std::cerr << "Field Mapper thread started" << std::endl;
        return true;

    } catch (const std::exception& e) 
    {
        std::cerr << "Failed to start Field Mapper update thread: " << e.what() << std::endl;
        running = false;
        return false;
    }
}

void FieldMapper::stop() 
{
    running = false;
    if (update_thread && update_thread->joinable()) 
    {
        update_thread->join();
    }

    update_thread.reset();
    running = false;
    std::cerr << "Field Mapper thread stopped" << std::endl;
}

bool FieldMapper::restart() 
{
    stop();
    return start();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void FieldMapper::updateLoop() 
{
    std::cerr << "Field Mapper update loop started" << std::endl;
    int process_count = 0;
    std::chrono::time_point last_time = std::chrono::high_resolution_clock::now();
    std::chrono::time_point current_time = std::chrono::high_resolution_clock::now();

    while (running) 
    {
        if(!camera.isInitialized() || !camera.isRunning())
        {
            std::cerr << " Field mapper is waiting for Camera" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
        }
        else if(!extrin_intrin_init && camera.isInitialized())
        {
            setIntrinsicAndExtrinsic();
        }
        else 
        {
            if(processRGBDImage())
            {
                //computeOdometry();
            }
            process_count++;
            current_time = std::chrono::high_resolution_clock::now();

        }

        int64 elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();

        if (elapsed >= 1000)
        {
            PPS = static_cast<int>(process_count * 1000 / elapsed);
            process_count = 0;
            last_time = current_time;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(35));

    }
    
    std::cerr << "Field Mapper update loop stopped" << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


// bool FieldMapper::processPointCloud() 
// {   
//     // Grab Raw Point Cloud from camera 
//     std::shared_ptr<open3d::geometry::PointCloud> raw_point_cloud = camera.getPointCloud();
    
//     if (raw_point_cloud == nullptr || raw_point_cloud->IsEmpty()) 
//     {
//         std::cerr << "Received empty point cloud from camera" << std::endl;
//         return false;
//     }
//     else 
//     {
//         std::lock_guard<std::mutex> lock(data_mutex);
//         legacy_point_cloud = raw_point_cloud->VoxelDownSample(0.01);
//         std::cerr << "Downsampled from " << raw_point_cloud->points_.size() 
//                   << " to " << legacy_point_cloud->points_.size() << " points" << std::endl;
//         tensor_point_cloud = open3d::t::geometry::PointCloud::FromLegacy(*legacy_point_cloud, 
//                                                                          open3d::core::Float32, 
//                                                                          open3d::core::Device("CPU:0"));
//         // current_tensor_cloud.EstimateNormals(30,1.5);
//         return true;
//     }      
// }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool FieldMapper::processRGBDImage() 
{   
    
    std::shared_ptr<open3d::geometry::RGBDImage> legacy_image = camera.getRGBDImage();

    if (legacy_image == nullptr || legacy_image->IsEmpty()) 
        return false;
    else
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        prevoius_image = current_image.Clone();
        open3d::t::geometry::Image color_image = open3d::t::geometry::Image::FromLegacy(legacy_image->color_, open3d::core::Device("CUDA:0"));
        open3d::t::geometry::Image depth_image = open3d::t::geometry::Image::FromLegacy(legacy_image->depth_, open3d::core::Device("CUDA:0"));

        current_image = open3d::t::geometry::RGBDImage(color_image, depth_image, true);
        
        if (current_image.IsEmpty())
            return false;
        else
            return true;

    }

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void FieldMapper::computeOdometry() 
{
    if (prevoius_image.IsEmpty() || current_image.IsEmpty()) 
    {
        return;
    }
    
    // Set up odometry parameters
    open3d::t::pipelines::odometry::OdometryConvergenceCriteria criteria(20, 1e-4, 1e-4);
    open3d::t::pipelines::odometry::OdometryLossParams loss_params(0.07, 0.05);
    
    // Compute odometry using hybrid method
     open3d::t::pipelines::odometry::OdometryResult odom_result = 
     open3d::t::pipelines::odometry::ComputeOdometryResultHybrid(prevoius_image, 
                                                                 current_image,
                                                                 intrinsic_tensor,
                                                                 extrinsic_tensor,  
                                                                 criteria,
                                                                 loss_params);
    
    // Process the odometry result
    if (odom_result.fitness_ > 0.3) 
    {  
        std::cerr << "Odometry success - fitness: " << odom_result.fitness_ 
                  << ", RMSE: " << odom_result.inlier_rmse_ << std::endl;
        
   
    } 
    // Here you would update pose graph, position, etc.
    // Example: current_pose = previous_pose * result.transformation_;
    else 
    {
        std::cerr << "Low quality odometry - fitness: " << odom_result.fitness_ 
                  << ", RMSE: " << odom_result.inlier_rmse_ << std::endl;
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool FieldMapper::segmentPlanes()
{
    std::lock_guard<std::mutex> lock(data_mutex);

    open3d::t::geometry::PointCloud processing_cloud = tensor_point_cloud.To(open3d::core::Device("CUDA:0"), true);
    
    std::vector<std::tuple<Eigen::Vector4d, open3d::core::Tensor>> potential_walls;
    std::tuple<open3d::core::Tensor, open3d::core::Tensor> segment_result;

        
    const int max_planes = 3;
    bool ground_plane_found = false;

    for (int i = 0; i < max_planes; ++i) 
    {
        if (processing_cloud.GetPointPositions().GetLength() < plane_ransac_n)
        {
            break;
        }
        try{
            
            segment_result = processing_cloud.SegmentPlane(
                                                    plane_distance_threshold,                  // Maximum distance from the plane
                                                    plane_ransac_n,                            // Number of points to sample
                                                    plane_num_iterations);                     // Number of RANSAC iterations
        } catch (const std::exception& e) 
        {
            std::cerr << "Failed to segment a plane from pointcloud " << e.what() << std::endl;
            break;
        }

        open3d::core::Tensor plane_tensor = std::get<0>(segment_result);
        open3d::core::Tensor plane_indices = std::get<1>(segment_result);

        // Extract plane equation (ax + by + cz + d = 0)
        float* data_ptr = static_cast<float*>(plane_tensor.GetDataPtr());
        Eigen::Vector4d plane_equation(data_ptr[0], data_ptr[1], data_ptr[2], data_ptr[3]);
        Eigen::Vector3d plane_normal(plane_equation[0], plane_equation[1], plane_equation[2]);

        plane_normal.normalize();
        float dot_product = plane_normal.dot(Eigen::Vector3d(0, 0, 1));
      
        if(!ground_plane_found && dot_product > ground_threshold) 
        {
            ground_plane = std::make_tuple(plane_equation, plane_indices);
            ground_plane_found = true;
        }
        else
        {
            potential_walls.push_back(std::make_tuple(plane_equation, plane_indices));
        }
       
        processing_cloud = processing_cloud.SelectByIndex(plane_indices, true);
    }

    if (ground_plane_found) 
    {
        std::cerr << "Found Ground Plane and " << potential_walls.size() << " potential walls" << std::endl;
        //identifyWalls(potential_walls);
    }
    else
    {
        std::cerr << "No Ground Plane detected" << std::endl;
    }
    
    return ground_plane_found;

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void FieldMapper::setIntrinsicAndExtrinsic()
{
    Eigen::Matrix3d intrinsic_matrix = camera.intrinsic.intrinsic_matrix_;
    Eigen::Matrix4d extrinsic_matrix = camera.extrinsic;

    
    std::lock_guard<std::mutex> lock(data_mutex);
    intrinsic_tensor = open3d::core::eigen_converter::EigenMatrixToTensor(intrinsic_matrix);
    extrinsic_tensor = open3d::core::eigen_converter::EigenMatrixToTensor(extrinsic_matrix);
    extrin_intrin_init = true;
    std::cerr << "Set extrinsic and intrinsic tensors ready for processing" << std::endl;


}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


int FieldMapper::getPPS()
{
    std::lock_guard<std::mutex> lock(data_mutex);
    return PPS;
}