#ifndef BRAIN_COMM_H
#define BRAIN_COMM_H

#include "BrainCommTypes.h"
#include "BrainCommStats.h"
#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include <iostream>
#include <queue>


namespace Brain {

class BrainComm {
public:
    explicit BrainComm(boost::asio::io_service& service);
    ~BrainComm();

    // Delete copy constructor and assignment operator
    BrainComm(const BrainComm&) = delete;
    BrainComm& operator=(const BrainComm&) = delete;

    // Core operations
    bool start();
    void stop();
    bool restart();
    // Request management
    bool updateRequests(uint16_t flags);       

    // Data setting
    void setMotorVoltages(float left, float right);
    void setMacroBits(uint32_t macro_bits);
    void setJetsonBattery(uint32_t level);

    // Data retrieval
    Position2D getLeftGPSData() const;
    Position2D getRightGPSData() const;
    Position2D getSisterPosition() const;
    Position2D getLeftGPSOffset() const;
    Position2D getRightGPSOffset() const;
    uint32_t getBrainBattery() const;

    // Statistics and error handling
    const StatsManager& getStats() const { return stats; }
    StatsManager& getStats() { return stats; }
    uint16_t getCurrentResponse() const { return response_flags; }
    uint16_t getCurrentRequests() const { return request_flags; }

    // Status checks
    bool isConnected() const { return connected; }
    bool isRunning() const { return running; }
    bool isStarted() const { return started; }
    bool isInitialized() const { return initialized; }
   
private:
    // Thread functions
    void readLoop();
    void writeLoop();
    bool findPort();
    
    // Internal methods
    bool initializePort();
    bool reconnect();
    bool processReceivedData(uint16_t flags, const uint8_t* data, uint16_t length);
    
    // Message handling methods
    void sendResponse(uint16_t flags);
    void sendAcknowledgment(uint16_t flags, uint8_t status);
    void sendRequests(uint16_t flags);
    bool handleMessage(const uint8_t* buffer, size_t length);
    
    // Serial Port variables
    std::string port;
    boost::asio::io_service& io_service;
    std::unique_ptr<boost::asio::serial_port> serial_port;
    bool connected;
    bool running;
    bool initialized;
    bool started;
    
    // Thread management
    std::unique_ptr<std::thread> read_thread;
    std::unique_ptr<std::thread> write_thread;
   

    // Mutexes for thread safety
    mutable std::mutex data_mutex;
    std::mutex state_mutex;

    // Request management
    std::queue<uint16_t> pending_requests;
    std::queue<uint16_t> pending_acknowledgments;
    bool request_in_progress;
    uint32_t last_request_time;
    uint8_t request_retry_count;
    
    // Communication state
    uint16_t request_flags;                 // Current Jetson request flags
    uint16_t response_flags;                // Current Brain request flags
    uint32_t last_send_time;                // Timestamp of last data send
    uint32_t last_received_time;            // Timestamp of last received data
    
    // Storage for Data From Brain
    Position2D left_gps_position;
    Position2D left_gps_offset;
    Position2D right_gps_position;
    Position2D right_gps_offset;
    Position2D sister_position;
    uint32_t BrainBatteryLvl;

    // Data to send to Brain
    MotorCommand current_motor_command;
    ControlFlags current_control_flags;
    uint32_t current_battery_lvl;
    
    // Statistics and error tracking
    StatsManager stats;
};

} // namespace Brain

#endif































