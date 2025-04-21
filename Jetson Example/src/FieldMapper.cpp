#include <FieldMapper.h>


using namespace open3d::t;

FieldMapper::FieldMapper(Camera& Camera) // , RobotPosition& Position) 
    : camera(Camera),
      // robot_position(Position),
      running(false),
      initialized(false),
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
      depth_scale(0.0f)
     
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
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            continue;
        }
        
        if(processRGBDImage())
        {
            if(initialize())
            {
                computeOdometry();
            }
        }
        process_count++;
        current_time = std::chrono::high_resolution_clock::now();

        int64 elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();

        if (elapsed >= 1000)
        {
            PPS = static_cast<int>(process_count * 1000 / elapsed);
            process_count = 0;
            last_time = current_time;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cerr << "Field Mapper update loop stopped" << std::endl;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool FieldMapper::initialize()
{
    if(initialized)
    {
        return true;
    }
    if(!camera.isInitialized())
    {
        std::cerr << "Can not initialize the Field Mapper camera has not been initialized" << std::endl;
        initialized = false;
        return false;
    }

    depth_scale = camera.getDepthScale();
    intrinsic_tensor = open3d::core::eigen_converter::EigenMatrixToTensor(camera.getIntrinsicMatrix());
    setExtrinsic(0, 0, 0, 0, 0, 0);
    current_transform = open3d::core::Tensor::Eye(4,open3d::core::Float32,
                                                  open3d::core::Device("CUDA:0"));
    prevoius_transform = current_transform.Clone();
    std::cerr << "Field Mapper has been initialized" << std::endl;
    initialized = true;
    return true;
    
}


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


bool FieldMapper::computeOdometry() 
{
    if (prevoius_image.IsEmpty() || current_image.IsEmpty()) 
    {
        std::cerr << "Couldnt compute odometry either prevoius and current images are empty/null" << std::endl;
        return false;
    }
    if(!initialized)
    {
        std::cerr << "Couldnt compute odometry, Field Mapper has not been initialized" << std::endl;
        return false;
    }

    float max_depth = 3.0f;

    // Set up odometry parameters
    open3d::t::pipelines::odometry::OdometryConvergenceCriteria criteria(20, 1e-4, 1e-4);
    open3d::t::pipelines::odometry::OdometryLossParams loss_params =  open3d::t::pipelines::odometry::OdometryLossParams(0.07, 0.05, 0.1);
    open3d::t::pipelines::odometry::Method odomMethod = open3d::t::pipelines::odometry::Method::Hybrid;
    
    // Compute odometry using hybrid method
    prevoius_transform = current_transform.Clone();
    open3d::t::pipelines::odometry::OdometryResult odom_result; 
    odom_result = open3d::t::pipelines::odometry::RGBDOdometryMultiScale(prevoius_image, 
                                                                         current_image,
                                                                         intrinsic_tensor,
                                                                         prevoius_transform,
                                                                         1/depth_scale,
                                                                         max_depth,
                                                                         {criteria},
                                                                         odomMethod);

    current_transform = odom_result.transformation_.Clone();
    
    // // Process the odometry result
    // if (odom_result.fitness_ > 0.3) 
    // {  
    std::cerr << "Odometry success - fitness: " << odom_result.fitness_ 
              << ", RMSE: " << odom_result.inlier_rmse_ << std::endl;
        
   
    // } 
    // // Here you would update pose graph, position, etc.
    // // Example: current_pose = previous_pose * result.transformation_;
    // else 
    // {
    //     std::cerr << "Low quality odometry - fitness: " << odom_result.fitness_ 
    //               << ", RMSE: " << odom_result.inlier_rmse_ << std::endl;
    // }
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void FieldMapper::setExtrinsic(float roll, float pitch, float yaw, float x, float y, float z)
{
    float roll_rad = roll * M_PI / 180.0f;
    float pitch_rad = pitch * M_PI / 180.0f;
    float yaw_rad = yaw * M_PI / 180.0f;
    // Convert roll, pitch, yaw (in radians) to rotation matrix using Eigen
    Eigen::AngleAxisd rollAngle(roll_rad, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(pitch_rad, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yawAngle(yaw_rad, Eigen::Vector3d::UnitY());

    Eigen::Matrix3d rotation_matrix = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();

    Eigen::Matrix4d extrinsic_matrix = Eigen::Matrix4d::Identity();
    extrinsic_matrix.block<3, 3>(0, 0) = rotation_matrix;

    extrinsic_matrix(0, 3) = x;
    extrinsic_matrix(1, 3) = y;
    extrinsic_matrix(2, 3) = z;
    std::lock_guard<std::mutex> lock(data_mutex);
    extrinsic_tensor = open3d::core::eigen_converter::EigenMatrixToTensor(extrinsic_matrix);
    std::cerr << "Extrinsic matrix set with | Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw <<
                 ", X: "  << x << ", Y:" << y << ", Z:" << z  << std::endl;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


int FieldMapper::getPPS()
{
    std::lock_guard<std::mutex> lock(data_mutex);
    return PPS;
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


// bool FieldMapper::segmentPlanes()
// {
//     std::lock_guard<std::mutex> lock(data_mutex);

//     open3d::t::geometry::PointCloud processing_cloud = tensor_point_cloud.To(open3d::core::Device("CUDA:0"), true);
    
//     std::vector<std::tuple<Eigen::Vector4d, open3d::core::Tensor>> potential_walls;
//     std::tuple<open3d::core::Tensor, open3d::core::Tensor> segment_result;

        
//     const int max_planes = 3;
//     bool ground_plane_found = false;

//     for (int i = 0; i < max_planes; ++i) 
//     {
//         if (processing_cloud.GetPointPositions().GetLength() < plane_ransac_n)
//         {
//             break;
//         }
//         try{
            
//             segment_result = processing_cloud.SegmentPlane(
//                                                     plane_distance_threshold,                  // Maximum distance from the plane
//                                                     plane_ransac_n,                            // Number of points to sample
//                                                     plane_num_iterations);                     // Number of RANSAC iterations
//         } catch (const std::exception& e) 
//         {
//             std::cerr << "Failed to segment a plane from pointcloud " << e.what() << std::endl;
//             break;
//         }

//         open3d::core::Tensor plane_tensor = std::get<0>(segment_result);
//         open3d::core::Tensor plane_indices = std::get<1>(segment_result);

//         // Extract plane equation (ax + by + cz + d = 0)
//         float* data_ptr = static_cast<float*>(plane_tensor.GetDataPtr());
//         Eigen::Vector4d plane_equation(data_ptr[0], data_ptr[1], data_ptr[2], data_ptr[3]);
//         Eigen::Vector3d plane_normal(plane_equation[0], plane_equation[1], plane_equation[2]);

//         plane_normal.normalize();
//         float dot_product = plane_normal.dot(Eigen::Vector3d(0, 0, 1));
      
//         if(!ground_plane_found && dot_product > ground_threshold) 
//         {
//             ground_plane = std::make_tuple(plane_equation, plane_indices);
//             ground_plane_found = true;
//         }
//         else
//         {
//             potential_walls.push_back(std::make_tuple(plane_equation, plane_indices));
//         }
       
//         processing_cloud = processing_cloud.SelectByIndex(plane_indices, true);
//     }

//     if (ground_plane_found) 
//     {
//         std::cerr << "Found Ground Plane and " << potential_walls.size() << " potential walls" << std::endl;
//         //identifyWalls(potential_walls);
//     }
//     else
//     {
//         std::cerr << "No Ground Plane detected" << std::endl;
//     }
    
//     return ground_plane_found;

// }