#include <Camera.h>

Camera::Camera() 
: 
  align_to(RS2_STREAM_COLOR), 
  FPS(0),
  device(nullptr),
//   intrinsic(Eigen::Matrix4d::Identity()),
//   extrinsic(Eigen::Matrix4d::Identity()),
  depth_scale(0.0f),
  running(false),
  connected(false),
  initialized(false),
  frame_width(640),
  frame_height(480)
{
    start();
}


Camera::~Camera() 
{
    stop();
}


void Camera::initialize()
{
 
    if(device != nullptr)
    {

        pipe.set_device(device);

        // Configure the pipeline for color and depth streams
        config.enable_stream(RS2_STREAM_COLOR, frame_width, frame_height, RS2_FORMAT_BGR8, 30);
        config.enable_stream(RS2_STREAM_DEPTH, frame_width, frame_height, RS2_FORMAT_Z16, 30);
            
        // Retrieve the profiles of the pipeline and color stream
        rs2::pipeline_profile pipe_profile = pipe.start(config);
        rs2::stream_profile color_stream = pipe_profile.get_stream(RS2_STREAM_COLOR);
        rs2::video_stream_profile color_profile = color_stream.as<rs2::video_stream_profile>();
        setIntrinsic(color_profile.get_intrinsics());
        depth_scale = device->first<rs2::depth_sensor>().get_depth_scale();
        initialized = true;
        std::cerr << "Realsense camera initialized" << std::endl;
       
    }
    else 
    {
        initialized = false;
        std::cerr << "Realsense camera can not be initialized no device connected" << std::endl;
    }
   
}


bool Camera::start() {
    if (running) 
    {
        std::cerr << "Camera thread is already running" << std::endl; 
        return true;
    }
    else
    {
        try{
            running = true;
            update_thread = std::make_unique<std::thread>(&Camera::updateLoop, this);
            return true;

            }catch (const std::exception& e) 
            {           
                std::cerr << "Failed to start camera thread: " << e.what() << std::endl;
                running = false;
                return false;  // Return false if thread creation fails
            }
    }

}

void Camera::stop()
{
    running = false;
    
    if (update_thread && update_thread->joinable()) {
        update_thread->join();
    }
    update_thread.reset();

    pipe.stop();
    connected = false;

}

bool Camera::restart() {
    std::cerr << "Restarting camera..." << std::endl;
    stop();
    return start();
}

bool Camera::reconnect() 
{   
    std::cerr << "Attempting to connect Realsense camera..." << std::endl;
    return findDevice();
}

void Camera::updateLoop() 
{
    std::cerr << "Camera update loop started" << std::endl;
    int frame_count = 0;
    std::chrono::time_point last_time = std::chrono::high_resolution_clock::now();

    while(running)
    {


        if (!connected) 
        {
                reconnect();
                std::this_thread::sleep_for(RECONNECT_DELAY);
                continue;
        }
        else 
        {
            //std::cerr << "Getting Frames" << std::endl;
            try{
                rs2::frameset frameset = pipe.wait_for_frames();
                frameset = align_to.process(frameset);
                
                // Get color and depth frames
                std::lock_guard<std::mutex> lock(stream_mutex);
                color_frame = frameset.get_color_frame();
                depth_frame = frameset.get_depth_frame();
                frame_count++;
            
                std::chrono::high_resolution_clock::now();

                std::chrono::time_point current_time = std::chrono::high_resolution_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();

                if(elapsed >= 1000)
                {
                    FPS = static_cast<int>(frame_count * 1000 / elapsed);
                    frame_count = 0;
                    last_time = current_time;
                }
            
            
            }catch(const rs2::camera_disconnected_error e)
            {
                std::cerr << "Realsense error: " << e.what() << std::endl;
                connected = false;
                continue;
            }

        }
    }
}

bool Camera::findDevice()
{
    rs2::device_list devices = context.query_devices();
    std::lock_guard<std::mutex> lock(stream_mutex);

    if(devices.size() < 1)
    {
        if(initialized)
        {
            pipe.stop();
            std::cerr << "Could not detect prevously connected camera" << std::endl;
            connected = false;
        }
        else
        {
            std::cerr << "No realsense devices were detected unable to set a device" << std::endl;
            connected = false;
        }
    }
    else if(devices.size() > 1)
    {
        std::cerr << "Too many realsense devices were detected" << std::endl;
        connected = false;
    }
    else 
    {
        std::cerr << "Realsense detected " << std::endl;

        if(device == nullptr)
        {
            device = new rs2::device(devices[0]);
            connected = true;
            initialize();
        }
        else 
        {
            if(devices.contains(*device))
            {
              
                connected = true;
                pipe.start(config);
            }
            else
                connected = false;
        }
    }
    return connected;
}




void Camera::setIntrinsic(const rs2_intrinsics &intrinsics)
{
    float intrinsic_data[] = {intrinsics.fx, 0, intrinsics.ppx, 0,
                              intrinsics.fy, intrinsics.ppy, 0, 0, 1};  
    
    intrinsic = open3d::core::Tensor(intrinsic_data, {3,3},
                                     open3d::core::Dtype::Float32,
                                     open3d::core::Device("CPU:0"));

} 

void Camera::setExtrinsic(float roll, float pitch, float yaw, float x, float y, float z)
{
    // Convert roll, pitch, yaw (in radians) to rotation matrix using Eigen
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitY());
    
    Eigen::Matrix3f rotation_matrix = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
    
    // Create full 4x4 transformation matrix
    float extrinsic_data[16] = {
        rotation_matrix(0,0), rotation_matrix(0,1), rotation_matrix(0,2), x,
        rotation_matrix(1,0), rotation_matrix(1,1), rotation_matrix(1,2), y,
        rotation_matrix(2,0), rotation_matrix(2,1), rotation_matrix(2,2), z,
        0, 0, 0, 1
    };
    
    // Create the extrinsic tensor
    extrinsic = open3d::core::Tensor(
        extrinsic_data,
        {4, 4},
        open3d::core::Dtype::Float32,
        open3d::core::Device("CPU:0"));
}

void Camera::getInferFrame(std::vector<float> &output)
{
    if(!connected)
    {
        std::cerr << "Could not retrieve inference frame, camera is not connected" << std::endl;
    }

    else
    {
        std::lock_guard<std::mutex> lock(stream_mutex);
        cv::Mat color_mat = cv::Mat(cv::Size(frame_width, frame_height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    
        cv::Mat resized;
        cv::resize(color_mat, resized, cv::Size(320, 320), 0, 0, cv::INTER_CUBIC);

        // Convert to NCHW format, normalize to [0,1]
        cv::Mat blob;
        cv::dnn::blobFromImage(resized, blob, 1.0 / 255.0, cv::Size(320, 320), cv::Scalar(), true, false);

        // Ensure blob is in the correct shape
        if (blob.dims != 4 || blob.size[0] != 1 || blob.size[1] != 3 || blob.size[2] != 320 || blob.size[3] != 320)
        {
            std::cerr << "ERROR: blobFromImage() produced incorrect shape!" << std::endl;
            return;
        }
        std::memcpy(output.data(), blob.ptr<float>(0), 320 * 320 * 3 * sizeof(float));
    }
}

open3d::t::geometry::PointCloud Camera::getPointCloud()
{

    if(!connected)
    {
        std::cerr << "Could not retrieve point cloud, camera is not connected" << std::endl;
        return open3d::t::geometry::PointCloud();

    }
    else
    {
        std::lock_guard<std::mutex> lock(stream_mutex);

        // Create color tensor
        open3d::core::Tensor color_tensor = open3d::core::Tensor(
        static_cast<const uint8_t*>(color_frame.get_data()),
        {frame_height, frame_width, 3},
        open3d::core::Dtype::UInt8,
        open3d::core::Device("CPU:0"));

        // Create depth tensor
        open3d::core::Tensor depth_tensor = open3d::core::Tensor(
        static_cast<const uint16_t*>(depth_frame.get_data()),
        {frame_height, frame_width},
        open3d::core::Dtype::UInt16,
        open3d::core::Device("CPU:0"));

        open3d::t::geometry::RGBDImage RGBDImage(color_tensor, depth_tensor);
        
        return open3d::t::geometry::PointCloud::CreateFromRGBDImage(RGBDImage, 
                                                                    intrinsic, 
                                                                    extrinsic, 
                                                                    depth_scale,
                                                                    10.0f, 1,
                                                                    false);

    }
}

int Camera::getFPS()
{
    std::lock_guard<std::mutex> lock(stream_mutex);
    return FPS;
}

// cv::Mat Camera::convertFrameToMat(const rs2::frame &frame)
// {
//     // Get frame dimensions
//     std::lock_guard<std::mutex> lock(stream_mutex);
//     int width = frame.as<rs2::video_frame>().get_width();
//     int height = frame.as<rs2::video_frame>().get_height();
//     // Convert to OpenCV Mat
//     if(frame.get_profile().format() == RS2_FORMAT_BGR8) 
//         return cv::Mat(cv::Size(width, height), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
//     else if(frame.get_profile().format() == RS2_FORMAT_Z16) 
//         return cv::Mat(cv::Size(width, height), CV_16U, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
// }
