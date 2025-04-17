#include "RobotPosition.h"
#include "PortDetector.h"
using namespace std;

// In RobotPosition.cpp - Initialize offsets in constructor
RobotPosition::RobotPosition(Brain::BrainComm& VEX_Brain, IMU& IMU_Sensor, boost::asio::io_service& Boost_Service)
    : brain(VEX_Brain),
      imu(IMU_Sensor),
      service(Boost_Service),
      gps_identified(false),
      offsets_received(false),
      heading_offset_(0.0f),
      running(false),
      position_confidence(0.0f),
      current_velocity(0.0f),
      current_angular_velocity(0.0f)
{
    // Initialize position data
    current_position = Position();
    last_position = Position();

    // Initialize GPS offsets
    left_gps_offset = {0.0f, 0.0f, 0.0f};
    right_gps_offset = {0.0f, 0.0f, 0.0f};
    
    // Initialize timestamps with system_clock
    last_calibration_time = std::chrono::system_clock::now();
    last_velocity_update = std::chrono::system_clock::now();

    startGPSDevices();
    start();
    
}

RobotPosition::~RobotPosition() {
    // Stop update thread if running
    stop();

    if (gps1) gps1->stop();
    if (gps2) gps2->stop();
}

bool RobotPosition::start() 
{
    if(running)
    {
        std::cerr << "Robot Position update thread is already running" << std::endl; 
        return true;
    }

    try{  
        
        running = true;
        update_thread = make_unique<thread>(&RobotPosition::updateLoop, this);
        return true;

    } catch (const std::exception& e) 
    {
        std::cerr << "Failed to start Robot Position update thread: " << e.what() << std::endl;
        running = false;
        return false; 
    }
  
}

void RobotPosition::stop() 
{
    running = false;
    
    if (update_thread->joinable()) 
    {
        update_thread->join();
    }
    update_thread.reset();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool RobotPosition::startGPSDevices()
{
    std::vector<std::string> gps_ports = PortDetector::findGPSPorts(); // Find available GPS ports

    if (gps_ports.size() < 1) 
    {
        std::cerr << "Error: Not enough GPS devices found!" << std::endl;
        return false;
    }
    else if(gps_ports.size() > 2)
    {
        std::cerr << "Error: Not enough GPS devices found!" << std::endl;
        return false;
    }

    // Initialize GPS sensors
    std::cerr << "Initializing GPS1 on port: " << gps_ports[0] << std::endl;
    gps1 = std::make_unique<GPS>(service, gps_ports[0]);
    if (!gps1->start()) 
    {
        std::cerr << "Error: Failed to start GPS1 thread" << std::endl;
        return false;
    }
    else
        std::cerr << "Started GPS1 thread" << std::endl;


    
    std::cerr << "Initializing GPS2 on port: " << gps_ports[1] << std::endl;
    gps2 = std::make_unique<GPS>(service, gps_ports[1]);
    if (!gps2->start()) 
    {
        std::cerr << "Error: Failed to start GPS2 thread" << std::endl;
        return false;
    }
    else
        std::cerr << "Started GPS2 thread" << std::endl;

    // Wait for GPS to get initial readings
    std::cerr << "Waiting for GPS readings..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void RobotPosition::updateLoop() 
{
    std::chrono::duration update_period = std::chrono::milliseconds(1000 / update_frequency);
    std::chrono::time_point start_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration elapsed = std::chrono::high_resolution_clock::now() - start_time;
    
    while (running) 
    {
        if(gps1 && gps2)
        {
            if(!gps_identified)
            {
                identifyGPSSensors();
                std::this_thread::sleep_for(RETRY_DELAY);
                continue;
            }
        }
        else
        {
            startGPSDevices();
            std::this_thread::sleep_for(RETRY_DELAY);
            continue;
        }
    
        start_time = std::chrono::high_resolution_clock::now();

        // Get raw position data from GPS sensors
        GPSPosition left_pos = left_gps->getGPSposition();
        GPSPosition right_pos = right_gps->getGPSposition();
        float left_quality = left_pos.quality;
        float right_quality = right_pos.quality;

        // Get brain-reported positions (already have offsets applied)
        Brain::Position2D brain_left = brain.getLeftGPSData();
        Brain::Position2D brain_right = brain.getRightGPSData();

        // Create a new position estimate
        Position new_position;

        // Calculate weighted position based on quality
        float total_quality = left_quality + right_quality;

        if (total_quality > 0.1f) 
        {
            // At least one GPS has some usable data
            float left_weight = left_quality / total_quality;
            float right_weight = right_quality / total_quality;
            
            // Use brain-reported positions (with offsets already applied)
            // This is more accurate than applying offsets ourselves
            new_position.x = brain_left.x * left_weight + brain_right.x * right_weight;
            new_position.y = brain_left.y * left_weight + brain_right.y * right_weight;
            
            // Z coordinate typically less important for 2D navigation
            new_position.z = (left_pos.z * left_weight + right_pos.z * right_weight);
            
            // Use IMU for heading (azimuth)
            new_position.azimuth = getHeadingFromIMU();
            
            // For elevation and rotation, use weighted average from GPS
            new_position.elevation = left_pos.elevation * left_weight + right_pos.elevation * right_weight;
            new_position.rotation = left_pos.rotation * left_weight + right_pos.rotation * right_weight;
            
            // Set confidence based on GPS quality
            new_position.confidence = total_quality / 2.0f;  // Average quality
        }
        else 
        {
            // No reliable GPS data - use last known position
            std::lock_guard<std::mutex> lock(position_mutex);
            new_position = current_position;
            // Still update heading from IMU
            new_position.azimuth = getHeadingFromIMU();
            // Decay confidence over time
            new_position.confidence *= 0.0f;
        }
        
        
        new_position.timestamp = std::chrono::high_resolution_clock::now(); // Set timestamp
        
       
        filterPosition(new_position);                       // Apply filtering
        updateVelocity(new_position);                       // Update velocity estimation
    
        // Update stored position
        {
            std::lock_guard<std::mutex> lock(position_mutex);
            current_position = new_position;
            position_confidence = new_position.confidence;
        }
        // Calculate time to sleep using steady_clock for consistent timing
        elapsed = std::chrono::high_resolution_clock::now() - start_time;
        if (elapsed < update_period) 
        {
            std::this_thread::sleep_for(update_period - elapsed);
        } 
        else 
        {
            // Log if we're not keeping up with desired frequency
            std::cerr << "Position update took longer than period: " 
                      << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() 
                      << "ms (target: " << update_period.count() << "ms)" << std::endl;
        }
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool RobotPosition::identifyGPSSensors() 
{

    if(!brain.isConnected() || !brain.isStarted())
    {
        return false;
    }

    // Check if we have received offsets yet
    if (!offsets_received) 
    {
        // Request GPS offsets if not already received
        brain.updateRequests(static_cast<uint16_t>(Brain::RequestFlag::LeftGPSData) | static_cast<uint16_t>(Brain::RequestFlag::RightGPSData));
        std::this_thread::sleep_for(std::chrono::milliseconds(750));

        uint16_t request_flags = brain.getCurrentRequests();

        if(request_flags != static_cast<uint16_t>(Brain::RequestFlag::LeftGPSData) | static_cast<uint16_t>(Brain::RequestFlag::RightGPSData))
        {
            std::cerr << "The request flags didnt match what was requested cannot identify GPS sensors" << std::endl;
            return false;
        }
        
        // Get current offsets from Brain
        left_gps_offset = brain.getLeftGPSOffset();
        right_gps_offset = brain.getRightGPSOffset();
        offsets_received = true;
        
        std::cerr << "Retrieved GPS offsets from Brain:" << std::endl;
        std::cerr << "Left GPS offset: (" << left_gps_offset.x << ", " 
                  << left_gps_offset.y <<  ", " << left_gps_offset.heading << ")" << std::endl;
        std::cerr << "Right GPS offset: (" << right_gps_offset.x << ", " 
                  <<  right_gps_offset.y <<  ", " <<  right_gps_offset.heading << ")" << std::endl;
    } 

    
    // Try multiple samples for more reliable identification
    const int NUM_SAMPLES = 50;
    std::vector<GPSPosition> gps1_samples;
    std::vector<GPSPosition> gps2_samples;
    std::vector<Brain::Position2D> brain_left_samples;
    std::vector<Brain::Position2D> brain_right_samples;
    
    std::cerr << "Collecting position samples for GPS identification..." << std::endl;
    
    // Collect samples
    for (int i = 0; i < NUM_SAMPLES; i++) 
    {
        gps1_samples.push_back(gps1->getGPSposition());
        gps2_samples.push_back(gps2->getGPSposition());
        brain_left_samples.push_back(brain.getLeftGPSData());
        brain_right_samples.push_back(brain.getRightGPSData());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cerr << "Collected position samples, identifying GPS sensors." << std::endl;
    
    // Calculate average positions
    GPSPosition avg_gps1;
    avg_gps1.x = 0;
    avg_gps1.y = 0;
    avg_gps1.azimuth = 0;

    for (int i = 0;  i < NUM_SAMPLES; i++) 
    {
        avg_gps1.x += gps1_samples[i].x;
        avg_gps1.y += gps1_samples[i].y;
        avg_gps1.azimuth += gps1_samples[i].azimuth;
    }

    avg_gps1.x /= NUM_SAMPLES;
    avg_gps1.y /= NUM_SAMPLES;
    avg_gps1.azimuth /= NUM_SAMPLES;

    GPSPosition avg_gps2;
    avg_gps2.x = 0;
    avg_gps2.y = 0;
    avg_gps2.azimuth = 0;

    for (int i = 0;  i < gps2_samples.size(); i++) 
    {
        avg_gps2.x += gps2_samples[i].x;
        avg_gps2.y += gps2_samples[i].y;
        avg_gps2.azimuth += gps2_samples[i].azimuth;
    }

    avg_gps2.x /= NUM_SAMPLES;
    avg_gps2.y /= NUM_SAMPLES;
    avg_gps2.azimuth /= NUM_SAMPLES;

    
    Brain::Position2D avg_brain_left;
    avg_brain_left.x = 0;
    avg_brain_left.y = 0;
    avg_brain_left.heading = 0;
    
    for (int i = 0;  i < NUM_SAMPLES; i++) 
    {
        avg_brain_left.x += (brain_left_samples[i].x -  left_gps_offset.x);
        avg_brain_left.y += (brain_left_samples[i].y - left_gps_offset.y);
        avg_brain_left.heading += (brain_left_samples[i].heading - left_gps_offset.heading);
    }

    avg_brain_left.x /= NUM_SAMPLES;
    avg_brain_left.y /= NUM_SAMPLES;
    avg_brain_left.heading /= NUM_SAMPLES;


    Brain::Position2D avg_brain_right;
    avg_brain_right.x = 0;
    avg_brain_right.y = 0;
    avg_brain_right.heading = 0;
    
    for (int i = 0;  i < NUM_SAMPLES; i++) 
    {
        avg_brain_right.x += (brain_right_samples[i].x -  right_gps_offset.x);
        avg_brain_right.y += (brain_right_samples[i].y - right_gps_offset.y);
        avg_brain_right.heading += (brain_right_samples[i].heading - right_gps_offset.heading);
    }

    avg_brain_right.x /= NUM_SAMPLES;
    avg_brain_right.y /= NUM_SAMPLES;
    avg_brain_right.heading /= NUM_SAMPLES;
    
    // Calculate distances to identify sensors
    float dist1_to_brain_left = std::sqrt(
        std::pow(avg_gps1.x - avg_brain_left.x, 2) + 
        std::pow(avg_gps1.y - avg_brain_left.y, 2));
    
    float dist2_to_brain_left = std::sqrt(
        std::pow(avg_gps2.x - avg_brain_left.x, 2) + 
        std::pow(avg_gps2.y - avg_brain_left.y, 2));

    float dist1_to_brain_right = std::sqrt(
        std::pow(avg_gps1.x - avg_brain_right.x, 2) + 
        std::pow(avg_gps1.y - avg_brain_right.y, 2));
    
    float dist2_to_brain_right = std::sqrt(
        std::pow(avg_gps2.x - avg_brain_right.x, 2) + 
        std::pow(avg_gps2.y - avg_brain_right.y, 2));
    
    std::cerr << "Distance from GPS1 to inferred left position: " << dist1_to_brain_left << std::endl;
    std::cerr << "Distance from GPS2 to inferred left position: " << dist2_to_brain_left << std::endl;
     std::cerr << "Distance from GPS1 to inferred right position: " << dist1_to_brain_right << std::endl;
    std::cerr << "Distance from GPS2 to inferred right position: " << dist2_to_brain_right << std::endl;
    
    // Determine which GPS is which based on closest match
    if (dist1_to_brain_left < dist2_to_brain_left && dist2_to_brain_right < dist1_to_brain_right) 
    {
        std::cerr << "GPS1 identified as Left sensor" << std::endl;
        std::cerr << "GPS2 identified as Right sensor" << std::endl;
        left_gps = gps1.get();
        right_gps = gps2.get();
    } 
    else 
    {
        std::cerr << "GPS1 identified as Right sensor" << std::endl;
        std::cerr << "GPS2 identified as Left sensor" << std::endl;
        left_gps = gps2.get();
        right_gps = gps1.get();
    }
    
    gps_identified = true;
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Position RobotPosition::averagePositions(const std::vector<GPSPosition>& positions) {
//     if (positions.empty()) {
//         return Position();
//     }
    
//     float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
//     float sin_sum_azimuth = 0.0f, cos_sum_azimuth = 0.0f;
//     float sin_sum_elevation = 0.0f, cos_sum_elevation = 0.0f;
//     float sin_sum_rotation = 0.0f, cos_sum_rotation = 0.0f;
    
//     for (const auto& pos : positions) {
//         sum_x += pos.x;
//         sum_y += pos.y;
//         sum_z += pos.z;
        
//         // For angular values, use sine/cosine averaging
//         float az_rad = pos.azimuth * M_PI / 180.0f;
//         float el_rad = pos.elevation * M_PI / 180.0f;
//         float rot_rad = pos.rotation * M_PI / 180.0f;
        
//         sin_sum_azimuth += std::sin(az_rad);
//         cos_sum_azimuth += std::cos(az_rad);
//         sin_sum_elevation += std::sin(el_rad);
//         cos_sum_elevation += std::cos(el_rad);
//         sin_sum_rotation += std::sin(rot_rad);
//         cos_sum_rotation += std::cos(rot_rad);
//     }
    
//     size_t count = positions.size();
    
//     // Calculate average angles
//     float avg_azimuth = std::atan2(sin_sum_azimuth, cos_sum_azimuth) * 180.0f / M_PI;
//     if (avg_azimuth < 0) avg_azimuth += 360.0f;
    
//     float avg_elevation = std::atan2(sin_sum_elevation, cos_sum_elevation) * 180.0f / M_PI;
//     float avg_rotation = std::atan2(sin_sum_rotation, cos_sum_rotation) * 180.0f / M_PI;
    
//     return Position(
//         sum_x / count,
//         sum_y / count,
//         sum_z / count,
//         avg_azimuth,
//         avg_elevation,
//         avg_rotation
//     );
// }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool RobotPosition::isPositionValid(const Position& position) const {
    // Check if coordinates are within reasonable bounds
    if (std::abs(position.x) > MAX_COORDINATE || 
        std::abs(position.y) > MAX_COORDINATE || 
        std::abs(position.z) > MAX_COORDINATE) {
        return false;
    }
    
    // Check if orientation angles are within expected ranges
    if (position.azimuth < 0.0f || position.azimuth >= 360.0f ||
        std::abs(position.elevation) > 90.0f || 
        std::abs(position.rotation) > 180.0f) {
        return false;
    }
    
    // Check for sudden jumps if we have position history
    if (!position_history.empty()) {
        const Position& prev = position_history.back();
        
        if (detectPositionJump(position, prev)) {
            return false;
        }
    }
    
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool RobotPosition::detectPositionJump(const Position& current, const Position& previous) const {
    // Check for position jumps
    float distance = current.distanceTo2D(previous);
    if (distance > MAX_POSITION_JUMP) {
        return true;
    }
    
    // Check for angle jumps
    float heading_diff = std::abs(current.azimuth - previous.azimuth);
    if (heading_diff > 180.0f) {
        heading_diff = 360.0f - heading_diff;
    }
    
    if (heading_diff > MAX_ANGLE_JUMP) {
        return true;
    }
    
    return false;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void RobotPosition::filterPosition(Position& position) {
    // Add to history
    position_history.push_back(position);
    if (position_history.size() > position_history_max_size) {
        position_history.pop_front();
    }
    
    // Need at least 3 points for filtering
    if (position_history.size() < 3) {
        return;
    }
    
    // Apply weighted average filter
    float x_sum = 0.0f, y_sum = 0.0f, z_sum = 0.0f;
    float sin_sum = 0.0f, cos_sum = 0.0f;
    float weight_sum = 0.0f;
    
    for (size_t i = 0; i < position_history.size(); i++) {
        // More weight to newer readings and higher confidence
        float age_factor = 0.6f + 0.4f * i / (position_history.size() - 1);
        float weight = age_factor * position_history[i].confidence;
        
        x_sum += position_history[i].x * weight;
        y_sum += position_history[i].y * weight;
        z_sum += position_history[i].z * weight;
        
        float heading_rad = position_history[i].azimuth * M_PI / 180.0f;
        sin_sum += std::sin(heading_rad) * weight;
        cos_sum += std::cos(heading_rad) * weight;
        
        weight_sum += weight;
    }
    
    // Apply filtered values if we have valid weights
    if (weight_sum > 0.001f) {
        position.x = x_sum / weight_sum;
        position.y = y_sum / weight_sum;
        position.z = z_sum / weight_sum;
        
        // Process heading
        float heading = std::atan2(sin_sum, cos_sum) * 180.0f / M_PI;
        if (heading < 0) heading += 360.0f;
        
        // Only update heading from filter if it's not too different from IMU
        float heading_diff = std::abs(heading - position.azimuth);
        if (heading_diff > 180.0f) {
            heading_diff = 360.0f - heading_diff;
        }
        
        // If filter heading is close to IMU heading, use it (smooths small variations)
        if (heading_diff < 15.0f) {
            position.azimuth = heading;
        }
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void RobotPosition::updateVelocity(const Position& current_position) 
{
    auto now = std::chrono::system_clock::now();  // Changed to system_clock
    
    // Calculate time difference in seconds
    float dt = std::chrono::duration<float>(now - last_velocity_update).count();
    
    if (dt > 0.001f) {  // Avoid division by very small numbers
        // Calculate linear velocity (distance/time)
        float distance = last_position.distanceTo2D(current_position);
        current_velocity = distance / dt;
        
        // Calculate angular velocity (heading change/time)
        float heading_diff = current_position.azimuth - last_position.azimuth;
        // Normalize heading difference to -180 to 180
        while (heading_diff > 180) heading_diff -= 360;
        while (heading_diff < -180) heading_diff += 360;
        current_angular_velocity = heading_diff / dt;
        
        // Update last position and time
        last_position = current_position;
        last_velocity_update = now;
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


float RobotPosition::getHeadingFromIMU() const 
{
    float mx, my, mz;
    if (imu.getMagnetometer(mx, my, mz)) 
    {
        // Calculate heading from magnetometer
        float heading = atan2(my, mx) * 180.0f / M_PI;
        
        // Normalize to 0-360
        while (heading < 0) heading += 360.0f;
        while (heading >= 360) heading -= 360.0f;
        
        // Apply calibration offset
        heading += heading_offset_;
        while (heading < 0) heading += 360.0f;
        while (heading >= 360) heading -= 360.0f;
        
        return heading;
    }
    
    // If IMU read fails, return last known heading
    std::lock_guard<std::mutex> lock(position_mutex);
    return current_position.azimuth;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool RobotPosition::isRobotStationary() const {
    // Read accelerometer and gyroscope
    float ax, ay, az, gx, gy, gz;
    if (!imu.getAccelerometer(ax, ay, az) || !imu.getGyroscope(gx, gy, gz)) {
        return false;
    }
    
    // Check if acceleration is close to gravity only
    float accel_magnitude = std::sqrt(ax*ax + ay*ay + az*az);
    if (std::abs(accel_magnitude - 1.0f) > 0.05f) {
        return false;
    }
    
    // Check if gyroscope readings are close to zero
    float gyro_magnitude = std::sqrt(gx*gx + gy*gy + gz*gz);
    if (gyro_magnitude > 1.0f) {
        return false;
    }
    
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


Position RobotPosition::getPosition() const {
    std::lock_guard<std::mutex> lock(position_mutex);
    return current_position;
}

void RobotPosition::get2DPosition(float& x, float& y, float& heading) const {
    std::lock_guard<std::mutex> lock(position_mutex);
    x = current_position.x;
    y = current_position.y;
    heading = current_position.azimuth;
}

float RobotPosition::getHeading() const {
    std::lock_guard<std::mutex> lock(position_mutex);
    return current_position.azimuth;
}

bool RobotPosition::hasReliablePosition() const {
    std::lock_guard<std::mutex> lock(position_mutex);
    return position_confidence >= MIN_CONFIDENCE_THRESHOLD;
}

float RobotPosition::getVelocity() const {
    return current_velocity;
}

float RobotPosition::getAngularVelocity() const {
    return current_angular_velocity;
}