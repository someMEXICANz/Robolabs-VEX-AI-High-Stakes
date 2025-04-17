#ifndef ROBOT_POSITION_H
#define ROBOT_POSITION_H

#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <deque>
#include "GPS.h"
#include "IMU.h"
#include "BrainComm.h"
#include "Position.h"

class RobotPosition {
public:
    // Constructor and initialization
    explicit RobotPosition(Brain::BrainComm& VEX_Brain, IMU& IMU_Sensor, boost::asio::io_service& Boost_Service);
    ~RobotPosition();

    // Delete copy constructor and assignment operator
    RobotPosition(const RobotPosition&) = delete;
    RobotPosition& operator=(const RobotPosition&) = delete;

    // Core operations
    bool initialize();
    bool start();
    void stop();
    bool isRunning() const { return running; }
    
    // Position access methods
    Position getPosition() const;
    void get2DPosition(float& x, float& y, float& heading) const;
    float getHeading() const;
    bool hasReliablePosition() const;  // Returns true if confidence > threshold
    
    // Motion data access
    float getVelocity() const;
    float getAngularVelocity() const;
    
    // Calibration and diagnostics
    void calibrateIMUHeading();
    bool areGPSIdentified() const { return gps_identified; }
    
private:
    // References to external components
    Brain::BrainComm& brain;
    IMU& imu;
    boost::asio::io_service& service;
    
    // GPS sensors managed by this class
    std::unique_ptr<GPS> gps1;
    std::unique_ptr<GPS> gps2;
    
    // Pointers to identified sensors (not owned)
    GPS* left_gps = nullptr;
    GPS* right_gps = nullptr;
    bool gps_identified = false;
    
    // Current position state
    Position current_position;
    float position_confidence = 0.0f;

    // GPS offsets from Brain
    Brain::Position2D left_gps_offset;
    Brain::Position2D right_gps_offset;
    bool offsets_received = false;
    
    // Heading calibration
    float heading_offset_ = 0.0f;
    std::chrono::system_clock::time_point last_calibration_time;  
    
    // Thread safety
    mutable std::mutex position_mutex;
    
    // Update thread
    std::unique_ptr<std::thread> update_thread;
    bool running;
    int update_frequency = 50;
  
    
    // Position filtering and processing
    std::deque<Position> position_history;
    const size_t position_history_max_size = 20;
    
    // Velocity calculation
    Position last_position;
    std::chrono::system_clock::time_point last_velocity_update;  // Changed to system_clock
    float current_velocity = 0.0f;
    float current_angular_velocity = 0.0f;
    
    // Position validation parameters
    const float MAX_POSITION_JUMP = 0.25f;
    const float MAX_ANGLE_JUMP = 30.0f;
    const float MAX_COORDINATE = 100.0f;
    const float MIN_CONFIDENCE_THRESHOLD = 0.3f;  // Minimum confidence for reliable position
    const std::chrono::milliseconds RETRY_DELAY{2500};
    
    // Helper methods
    bool startGPSDevices();
    bool identifyGPSSensors();
    float getHeadingFromIMU() const;
    bool isRobotStationary() const;
    Position averagePositions(const std::vector<GPSPosition>& positions);
    void filterPosition(Position& position);
    bool detectPositionJump(const Position& current, const Position& previous) const;
    void updateVelocity(const Position& current_position);
    bool isPositionValid(const Position& position) const;
    
    // Thread function
    void updateLoop();
};

#endif // ROBOT_POSITION_H