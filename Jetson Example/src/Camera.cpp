#include <Camera.h>

Camera::Camera()
    : align_to(RS2_STREAM_COLOR),
      FPS(0),
      device(nullptr),
      depth_scale(0.0f),
      running(false),
      connected(false),
      initialized(false),
      frame_width(640),
      frame_height(480)
{
    connect();
    start();
}

Camera::~Camera()
{
    stop();
}

bool Camera::start()
{
    if (running)
    {
        std::cerr << "Camera thread is already running" << std::endl;
        return true;
    }
    else
    {
        try
        {
            running = true;
            update_thread = std::make_unique<std::thread>(&Camera::updateLoop, this);
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to start camera thread: " << e.what() << std::endl;
            running = false;
            return false; // Return false if thread creation fails
        }
    }
}

void Camera::stop()
{
    running = false;

    if (update_thread && update_thread->joinable())
    {
        update_thread->join();
        update_thread.reset();
    }

    pipe.stop();
    connected = false;
}

bool Camera::restart()
{
    std::cerr << "Restarting camera..." << std::endl;
    stop();
    return start();
}

bool Camera::connect()
{
    std::cerr << "Attempting to connect Realsense camera..." << std::endl;
    return findDevice();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void Camera::initialize()
{

    if (device == nullptr)
    {
        initialized = false;
        std::cerr << "Realsense camera can not be initialized no device connected" << std::endl;
    }
    else
    {
        pipe.set_device(device);
        // Configure the pipeline for color and depth streams
        config.enable_stream(RS2_STREAM_COLOR, frame_width, frame_height, RS2_FORMAT_BGR8, 30);
        config.enable_stream(RS2_STREAM_DEPTH, frame_width, frame_height, RS2_FORMAT_Z16, 30);

        // Retrieve the profiles of the pipeline and color stream
        rs2::pipeline_profile pipe_profile = pipe.start(config);
        rs2::stream_profile color_stream = pipe_profile.get_stream(RS2_STREAM_COLOR);
        rs2::video_stream_profile color_profile = color_stream.as<rs2::video_stream_profile>();
        rs2_intrinsics rs_intrinsics = color_profile.get_intrinsics();
        intrinsic = open3d::camera::PinholeCameraIntrinsic(frame_width, frame_height,
                                                           rs_intrinsics.fx, rs_intrinsics.fy,
                                                           rs_intrinsics.ppx, rs_intrinsics.ppy);

        depth_scale = device->first<rs2::depth_sensor>().get_depth_scale();
        std::cerr << "Depth Scale: " << depth_scale << std::endl;

        color_image = std::make_shared<open3d::geometry::Image>();
        depth_image = std::make_shared<open3d::geometry::Image>();
        color_image->Prepare(frame_width, frame_height, 3, 1);
        depth_image->Prepare(frame_width, frame_height, 1, 2);
        current_RGBDImage = std::make_shared<open3d::geometry::RGBDImage>();
        initialized = true;
        std::cerr << "Realsense camera initialized" << std::endl;
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool Camera::findDevice()
{
    rs2::device_list devices = context.query_devices();
    std::lock_guard<std::mutex> lock(stream_mutex);

    if (devices.size() < 1)
    {

        std::cerr << "No realsense devices were detected via USB unable to set a device" << std::endl;
        connected = false;
    }
    else if (devices.size() > 1)
    {
        std::cerr << "Too many realsense devices were detected" << std::endl;
        connected = false;
    }
    else
    {
        std::cerr << "Realsense detected " << std::endl;

        if (device == nullptr)
        {
            device = new rs2::device(devices[0]);
            connected = true;
            initialize();
        }
    }
    return connected;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void Camera::updateLoop()
{
    std::cerr << "Camera update loop started" << std::endl;
    int frame_count = 0;
    std::chrono::time_point last_time = std::chrono::high_resolution_clock::now();
    std::chrono::time_point current_time = std::chrono::high_resolution_clock::now();

    while (running)
    {

        if (!connected || !initialized)
        {
            connect();
            std::this_thread::sleep_for(RECONNECT_DELAY);
            continue;
        }
        else
        {

            rs2::frameset frameset = pipe.wait_for_frames();
            frameset = align_to.process(frameset);

            // Get color and depth frames
            std::lock_guard<std::mutex> lock(stream_mutex);
            color_frame = frameset.get_color_frame();
            depth_frame = frameset.get_depth_frame();

            frame_count++;
            current_time = std::chrono::high_resolution_clock::now();

            std::memcpy(color_image->data_.data(), color_frame.get_data(),
                        frame_width * frame_height * 3 );
            std::memcpy(depth_image->data_.data(), depth_frame.get_data(),
                        frame_width * frame_height * 2);

            current_RGBDImage = open3d::geometry::RGBDImage::CreateFromColorAndDepth(
                *color_image, *depth_image,
                1 / depth_scale, 10, false);
            

        }

        int64 elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();

        if (elapsed >= 1000)
        {
            FPS = static_cast<int>(frame_count * 1000 / elapsed);
            frame_count = 0;
            last_time = current_time;
        }
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


int Camera::getFPS()
{
    std::lock_guard<std::mutex> lock(stream_mutex);
    return FPS;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


float Camera::getDepthScale()
{
    if(!initialized)
    {
        std::cerr << "Could not retrieve depth scale, camera is not initialized" << std::endl;
    }
    else
    {
        std::lock_guard<std::mutex> lock(stream_mutex);
        return depth_scale;
    }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


Eigen::Matrix3d Camera::getIntrinsicMatrix()
{
    if(!initialized)
    {
        std::cerr << "Could not retrieve Intrinsic mastrix camera is not initialized" << std::endl;
    }
    else 
    {
        std::lock_guard<std::mutex> lock(stream_mutex);
        return intrinsic.intrinsic_matrix_;
    }

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool Camera::getInferFrame(std::vector<float> &output)
{
    if (!initialized || !running)
    {
        std::cerr << "Could not retrieve inference frame, camera is not connected" << std::endl;
        return false;
    }
    else
    {
        std::lock_guard<std::mutex> lock(stream_mutex);
        cv::Mat color_mat = cv::Mat(cv::Size(frame_width, frame_height), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);

        cv::Mat resized;
        cv::resize(color_mat, resized, cv::Size(320, 320), 0, 0, cv::INTER_CUBIC);

        // Convert to NCHW format, normalize to [0,1]
        cv::Mat blob;
        cv::dnn::blobFromImage(resized, blob, 1.0 / 255.0, cv::Size(320, 320), cv::Scalar(), true, false);

        // Ensure blob is in the correct shape
        if (blob.dims != 4 || blob.size[0] != 1 || blob.size[1] != 3 || blob.size[2] != 320 || blob.size[3] != 320)
        {
            std::cerr << "ERROR: blobFromImage() produced incorrect shape!" << std::endl;
            return false;
        }
        std::memcpy(output.data(), blob.ptr<float>(0), 320 * 320 * 3 * sizeof(float));
        return true;
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


std::shared_ptr<open3d::geometry::RGBDImage> Camera::getRGBDImage()
{
    std::lock_guard<std::mutex> lock(stream_mutex);
    if(!initialized || !running)
    {
        //std::cerr << "Could not retrieve RGBD Image, camera is not connected" << std::endl;
        return nullptr;
    }
    else if(current_RGBDImage->IsEmpty())
    {
        //std::cerr << "RGBD Image is empty" << std::endl;
        return nullptr;
    }
    else
    {
        return current_RGBDImage;
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


// std::shared_ptr<open3d::geometry::PointCloud> Camera::getPointCloud()
// {
//     std::lock_guard<std::mutex> lock(stream_mutex);
    
//     if (!initialized || !running)
//     {
//         std::cerr << "Could not generate point, camera is not connected" << std::endl;
//         return nullptr;
//     }
//     else if (current_RGBDImage->IsEmpty())
//     {
//         return nullptr;
//     }
//     else
//     {
//         std::shared_ptr<open3d::geometry::PointCloud> Point_Cloud = std::make_shared<open3d::geometry::PointCloud>();
//         Point_Cloud = open3d::geometry::PointCloud::CreateFromRGBDImage(*current_RGBDImage, intrinsic, extrinsic);
//         if (Point_Cloud->IsEmpty())
//         {
//             std::cerr << "Point Cloud is empty" << std::endl;
//             return nullptr;
//         }
//         else
//         {
//             //std::cerr << "Point Cloud has " << Point_Cloud->points_.size() << " points" << std::endl;
//             return Point_Cloud;
//         }
//     }
// }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


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


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


// void Camera::setExtrinsic(float roll, float pitch, float yaw, float x, float y, float z)
// {
//     float roll_rad = roll * M_PI / 180.0f;
//     float pitch_rad = pitch * M_PI / 180.0f;
//     float yaw_rad = yaw * M_PI / 180.0f;
//     // Convert roll, pitch, yaw (in radians) to rotation matrix using Eigen
//     Eigen::AngleAxisd rollAngle(roll_rad, Eigen::Vector3d::UnitZ());
//     Eigen::AngleAxisd pitchAngle(pitch_rad, Eigen::Vector3d::UnitX());
//     Eigen::AngleAxisd yawAngle(yaw_rad, Eigen::Vector3d::UnitY());

//     Eigen::Matrix3d rotation_matrix = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();

//     extrinsic = Eigen::Matrix4d::Identity();
//     extrinsic.block<3, 3>(0, 0) = rotation_matrix;

//     extrinsic(0, 3) = x;
//     extrinsic(1, 3) = y;
//     extrinsic(2, 3) = z;

//     std::cerr << "Extrinsic matrix set with | Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw <<
//                  " (X,Y,Z): (" << x << y << z << ")" << std::endl;
// }