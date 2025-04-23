#ifndef GPS_H
#define GPS_H

#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>
#include <cstring>
#include <iostream>

// Status bit definitions
static constexpr uint32_t STATUS_CONNECTED    = 0x00000001;
static constexpr uint32_t STATUS_NODOTS       = 0x00000002;
static constexpr uint32_t STATUS_NORAWBITS    = 0x00000004;
static constexpr uint32_t STATUS_NOGROUPS     = 0x00000008;
static constexpr uint32_t STATUS_NOBITS       = 0x00000010;
static constexpr uint32_t STATUS_PIXELERROR   = 0x00000020;
static constexpr uint32_t STATUS_SOLVER       = 0x00000040;
static constexpr uint32_t STATUS_ANGLEJUMP    = 0x00000080;
static constexpr uint32_t STATUS_POSJUMP      = 0x00000100;
static constexpr uint32_t STATUS_NOSOLUTION   = 0x00000200;
static constexpr uint32_t STATUS_KALMAN_EST   = 0x00100000;


struct GPSPosition
{

    float x, y, z;
    float azimuth, elevation, rotation;
    float quality; 
    std::chrono::high_resolution_clock::time_point timestamp;

};

class GPS 
{
public:

    explicit GPS(boost::asio::io_service& service, 
                const std::string& new_port = "");
    ~GPS();

    // Delete copy constructor and assignment operator
    GPS(const GPS&) = delete;
    GPS& operator=(const GPS&) = delete;

    // Core operations
    bool start();
    void stop();
    bool restart();

    // Status Checks
    bool isConnected() const { return connected; }
    bool isRunning() const { return running; }
    bool isInitialized() const { return initialized; }
    
    GPSPosition getPosition() const;

    // Configuration methods
    const std::string& getPort() const { return port; }

private:
    // Internal methods
    void readLoop();
    bool readData();

    bool connectPort();
    bool initializePort();
    bool reconnect();
    float calculateQuality(uint32_t current_status) const;
    
   

    // Serial Port variables
    std::string port;
    boost::asio::io_service& io_service;
    std::unique_ptr<boost::asio::serial_port> serial_port;
   
    
    // Thread safety and state
    mutable std::mutex position_mutex;
    std::unique_ptr<std::thread> read_thread;  
    GPSPosition current_position;
    bool running;
    bool connected;
    bool initialized;

    // Configuration constants
    const std::chrono::milliseconds RECONNECT_DELAY{2500};
};

#endif // GPS_H