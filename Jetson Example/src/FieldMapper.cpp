#include <FieldMapper.h>

FieldMapper::FieldMapper(Camera& Camera, RobotPosition& Position) 
    : camera(Camera),
      robot_position(Position),
      running(false),
      map_width(3.66f),
      map_height(3.66f),
      map_resolution(0.05f), 
      cluster_eps(0.2f),
      cluster_min_points(10),
      plane_distance_threshold(0.05f),
      plane_ransac_n(3),
      plane_num_iterations(1000),
      min_height_threshold(0.1f),
      max_height_threshold(1.5f)
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

    while (running) 
    {
        if(camera.isConnected())
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
    
    if (raw_point_cloud = nullptr || raw_point_cloud->IsEmpty()) 
    {
        std::cerr << "Received empty point cloud from camera" << std::endl;
        return false;
    }
    else 
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        legacy_point_cloud = raw_point_cloud->VoxelDownSample(0.01);
        std::cerr << "Downsampled from " << raw_point_cloud->points_.size() 
                  << " to " << legac->points_.size() << " points" << std::endl;
        current_tensor_cloud = open3d::t::geometry::PointCloud::FromLegacy(*legacy_cloud, 
                                                                           open3d::core::Float32, 
                                                                           open3d::core::Device("CPU:0"));
        current_tensor_cloud.EstimateNormals(30,1.5);
        return true;
    }      
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void FieldMapper::segmentPlanes()
{
    std::lock_guard<std::mutex> lock(data_mutex);
    std::vector<std::tuple<Eigen::Vector4d, open3d::geometry::PointCloud>> detected_planes;   
    const int max_planes = 3;


    for (int i = 0; i < max_planes; ++i) 
    {
        if (current_tensor_cloud.GetPointPositions().GetLength() < plane_ransac_n);
            break;
    
        // Perform plane segmentation using RANSAC
        std::tuple< open3d::core::Tensor, open3d::core::Tensor> 
        result = current_tensor_cloud.SegmentPlane(
                                            plane_distance_threshold,                  // Maximum distance from the plane
                                            plane_ransac_n,                            // Number of points to sample
                                            plane_num_iterations);                     // Number of RANSAC iterations
            
        open3d::core::Tensor plane_tensor = std::get<0>(result);
        open3d::core::Tensor plane_indices = std::get<1>(result);

        float* data_ptr = static_cast<float*>(plane_tensor.GetDataPtr());
        Eigen::Vector4d plane_equation(data_ptr[0], data_ptr[1], data_ptr[2], data_ptr[3]);

        open3d::t::geometry::PointCloud plane_cloud = current_tensor_cloud.SelectByIndex(plane_indices, false, true);

        std::tuple<Eigen::Vector4d, open3d::geometry::PointCloud> current_plane = std::make_tuple(plane_equation, plane_cloud.ToLegacy());
        
        detected_planes.push_back(current_plane);
    }
}
                
            