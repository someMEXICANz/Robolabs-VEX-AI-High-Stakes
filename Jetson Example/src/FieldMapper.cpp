#include <FieldMapper.h>

FieldMapper::FieldMapper(Camera& Camera) // , RobotPosition& Position) 
    : camera(Camera),
      // robot_position(Position),
      running(false),
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
      max_height_threshold(1.5f)
{

    legacy_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
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

    while (running) 
    {
        if(!camera.isConnected() && !camera.isInitialized())
        {
            std::cerr << " Field mapper is Waiting for Camera" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
        }
        else 
        {
            if (processPointCloud())
            {
                segmentPlanes();
            }
        }
    }
    
    std::cerr << "Field Mapper update loop stopped" << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool FieldMapper::processPointCloud() 
{   
    // Grab Raw Point Cloud from camera 
    std::shared_ptr<open3d::geometry::PointCloud> raw_point_cloud = camera.getPointCloud();
    
    if (raw_point_cloud == nullptr || raw_point_cloud->IsEmpty()) 
    {
        std::cerr << "Received empty point cloud from camera" << std::endl;
        return false;
    }
    else 
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        legacy_point_cloud = raw_point_cloud->VoxelDownSample(0.01);
        std::cerr << "Downsampled from " << raw_point_cloud->points_.size() 
                  << " to " << legacy_point_cloud->points_.size() << " points" << std::endl;
        tensor_point_cloud = open3d::t::geometry::PointCloud::FromLegacy(*legacy_point_cloud, 
                                                                         open3d::core::Float32, 
                                                                         open3d::core::Device("CPU:0"));
        // current_tensor_cloud.EstimateNormals(30,1.5);
        return true;
    }      
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


// bool FieldMapper::segmentPlanes()
// {
//     std::cerr << "Segmenting Point CLoud" << std::endl;
//     std::lock_guard<std::mutex> lock(data_mutex);
//     std::vector<std::tuple<Eigen::Vector4d, open3d::geometry::PointCloud>> detected_planes;   
//     const int max_planes = 3;

//     open3d::t::geometry::PointCloud temp_point_cloud = tensor_point_cloud.To(open3d::core::Device("CUDA:0"), true);


//     for (int i = 0; i < max_planes; ++i) 
//     {
//         if (temp_point_cloud.GetPointPositions().GetLength() < plane_ransac_n);
//             break;
    
//         // Perform plane segmentation using RANSAC
//         std::tuple< open3d::core::Tensor, open3d::core::Tensor> 
//         result = temp_point_cloud.SegmentPlane(
//                                             plane_distance_threshold,                  // Maximum distance from the plane
//                                             plane_ransac_n,                            // Number of points to sample
//                                             plane_num_iterations);                     // Number of RANSAC iterations
            
//         open3d::core::Tensor plane_tensor = std::get<0>(result);
//         open3d::core::Tensor plane_indices = std::get<1>(result);

//         float* data_ptr = static_cast<float*>(plane_tensor.GetDataPtr());
//         Eigen::Vector4d plane_equation(data_ptr[0], data_ptr[1], data_ptr[2], data_ptr[3]);

//         open3d::t::geometry::PointCloud plane_cloud = temp_point_cloud.SelectByIndex(plane_indices, false, true);

//         std::tuple<Eigen::Vector4d, open3d::geometry::PointCloud> current_plane = std::make_tuple(plane_equation, plane_cloud.ToLegacy());
        
//         detected_planes.push_back(current_plane);
//     }
// }



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
        if (processing_cloud.GetPointPositions().GetLength() < plane_ransac_n);
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
            