#include <Camera.h>

Camera::Camera() 
: align_to(RS2_STREAM_COLOR),
  frame_width(640),
  frame_height(480),
  running(false),
  connected(false),
  fps(0.0f)
{
    initialize();
    start();
}


Camera::~Camera() 
{
    stop();
}


bool Camera::initialize()
{
    try {
        // Configure the pipeline for color and depth streams
        config.enable_stream(RS2_STREAM_COLOR, frame_width, frame_height, RS2_FORMAT_BGR8, 30);
        config.enable_stream(RS2_STREAM_DEPTH, frame_width, frame_height, RS2_FORMAT_Z16, 30);

        // Retrieve the depth scale from a temporary pipeline start
        rs2::pipeline_profile pipe_profile = pipe.start(config);
        rs2::depth_sensor sensor = pipe_profile.get_device().first<rs2::depth_sensor>();

        rs_sensor = std::make_shared<rs2::depth_sensor>(sensor);
        depth_scale = rs_sensor->get_depth_scale();
        connected = true;
        depth_stream = pipe_profile.get_stream(RS2_STREAM_DEPTH);
        color_stream = pipe_profile.get_stream(RS2_STREAM_COLOR);
        depth_profile = depth_stream.as<rs2::video_stream_profile>();
        color_profile = color_stream.as<rs2::video_stream_profile>();

        rs_intrinsic = depth_profile.get_intrinsics();
        o3d_intrinsic.SetIntrinsics(rs_intrinsic.width, rs_intrinsic.height, 
                                     rs_intrinsic.fx, rs_intrinsic.fy, 
                                     rs_intrinsic.ppx, rs_intrinsic.ppy);
        o3d_extrinsic.s

        std::cerr << "Realsense camera connected and initialized" << std::endl;
        return true;
    }
    catch(const rs2::error& e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        connected = false;
        return false;
    }
    catch(const std::exception& e) {
        std::cerr << "Error initializing camera: " << e.what() << std::endl;
        connected = false;
        return false;
    }
}


bool Camera::start() {
    if (running) 
    {
        std::cerr << "Camera thread is already running" << std::endl; 
        return true;
    }

    // Try to establish camera connection if needed
    // if (!connected) {
    //     if (!initialize()) {
    //         std::cerr << "Error in start(): failed to initialize camera" << std::endl;
    //         return false;  // Return false if initialization fails
    //     }
    // }
    
    try {
        running = true;
        update_thread = std::make_unique<std::thread>(&Camera::updateLoop, this);
        return true;

    } catch (const std::exception& e) 
    {
        std::cerr << "Failed to start camera thread: " << e.what() << std::endl;
        running = false;
        return false;  // Return false if thread creation fails
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
    std::cerr << "Attempting to reconnect camera..." << std::endl;
    try{
        if (connected) 
        {
            pipe.stop();
        }
        connected = false;
    } catch (const std::exception& e) {
        std::cerr << "Error stopping pipeline during reconnect: " << e.what() << std::endl;
    }
    
    // Try to initialize again
    return initialize();
}

void Camera::updateLoop() 
{
    std::cerr << "Camera update loop started" << std::endl;
    while(running)
    {
        if (!connected) 
        {
            if(!reconnect())
            {
                std::this_thread::sleep_for(RECONNECT_DELAY);
                continue;
            }
        }
        updateStreams();
    
    }

}




cv::Mat Camera::convertFrameToMat(const rs2::frame &frame)
{
    // Get frame dimensions
    int width = frame.as<rs2::video_frame>().get_width();
    int height = frame.as<rs2::video_frame>().get_height();
    // Convert to OpenCV Mat
    if(frame.get_profile().format() == RS2_FORMAT_BGR8) 
        return cv::Mat(cv::Size(width, height), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
    else if(frame.get_profile().format() == RS2_FORMAT_Z16) 
        return cv::Mat(cv::Size(width, height), CV_16U, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
}



void Camera::updateStreams()
{
    rs2::frameset frameset = pipe.wait_for_frames();
    frameset = align_to.process(frameset);

    std::lock_guard<std::mutex> lock(stream_mutex);
    // Get color and depth frames
    color_frame = frameset.get_color_frame();
    depth_frame = frameset.get_depth_frame();
    depth_scale = rs_sensor->get_depth_scale();

    std::chrono::time_point currentTime = std::chrono::steady_clock::now();
    float deltaTime = std::chrono::duration<float>(currentTime - lastFrameTime).count();
    lastFrameTime = currentTime;
    float currentFPS = 1.0f / deltaTime;
    fps = (fps * (1.0f - 0.1f)) + (currentFPS * 0.1f);
    
    

}



void Camera::preprocessFrames(std::vector<float> &output)
{
    std::lock_guard<std::mutex> lock(stream_mutex);
    cv::Mat color_mat = convertFrameToMat(color_frame);
    if (color_mat.empty()) {
        std::cerr << "Error: color_mat is empty, cannot preprocess frames" << std::endl;
        return;
    }
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


std::shared_ptr<open3d::geometry::PointCloud> Camera::getPointCloud()
{
    std::lock_guard<std::mutex> lock(stream_mutex);
    const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
    open3d::geometry::Image depth_image;
    depth_image.Prepare(rs_intrinsic.width, rs_intrinsic.height, 1, 2);
    std::memcpy(depth_image.data_.data(), depth_data, rs_intrinsic.width * rs_intrinsic.height * 2);
    return open3d::geometry::PointCloud::CreateFromDepthImage(depth_image, 
                                                               o3d_intrinsic, 
                                                               o3d_extrinsic, 
                                                               depth_scale, 
                                                               1000, 1, true);
}


void Camera::displayDetections(const std::vector<DetectedObject>& detections)
{      

    std::lock_guard<std::mutex> lock(stream_mutex);
    cv::Mat color_mat = convertFrameToMat(color_frame);

    cv::Mat display_image = color_mat.clone();
    std::string fps_text = "FPS: " + std::to_string(static_cast<int>(fps));
    cv::putText(color_mat, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    
    for (const auto& det : detections) 
    {
        // Rest of your drawing code, but use scaled_bbox instead of det.bbox
        cv::Scalar color;
        std::string label;
        
        switch(det.classId) 
        {
            case 0:
                color = cv::Scalar(0, 255, 0);
                label = "MobileGoal";
                break;
            case 1:
                color = cv::Scalar(0, 0, 255);
                label = "RedRing";
                break;
            case 2:
                color = cv::Scalar(255, 0, 0);
                label = "BlueRing";
                break;
        }

        cv::rectangle(display_image, det.bbox, color, 2);
        label += " " + std::to_string(static_cast<int>(det.confidence * 100)) + "%";

        cv::Point text_origin(det.bbox.x, det.bbox.y - 5);
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, nullptr);
        cv::rectangle(display_image, 
                     cv::Point(text_origin.x, text_origin.y - text_size.height),
                     cv::Point(text_origin.x + text_size.width, text_origin.y + 5),
                     color, -1);

        cv::putText(display_image, label, text_origin,cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
    }

    cv::imshow("Color Frame", display_image);
 
}