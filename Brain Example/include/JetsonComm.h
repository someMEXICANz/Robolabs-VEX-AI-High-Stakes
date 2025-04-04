

#ifndef JETSON_COMM_H
#define JETSON_COMM_H

#include "JetsonCommTypes.h"
#include "JetsonCommStats.h"
#include <queue>
#include <stdatomic.h>
#include <condition_variable>

namespace Jetson {

class JetsonComm {
public:
    JetsonComm();
    ~JetsonComm();


    JetsonComm(const JetsonComm&) = delete;
    JetsonComm& operator=(const JetsonComm&) = delete;

    // Request management
    void updateRequests(uint16_t flags);
    
    // Getters for received commands
    MotorCommand getMotorCommand() const;
    ControlFlags getControlFlags() const;
    uint32_t getJetsonBattery() const;

    // Status checks
    const StatsManager& getStats() const { return stats; }
    StatsManager& getStats() { return stats; }
    uint16_t getCurrentResponse() const { return response_flags; }
    uint16_t getCurrentRequests() const { return request_flags; }

private:

    FILE* serial_port;
    // Thread management
    static int readThread(void* arg);
    static int writeThread(void* arg);
    
    // Internal methods
    bool processReceivedData(uint16_t flags, const uint8_t* data, uint16_t length);
    bool sendResponse(uint16_t flags);
    bool sendAcknowledgment(uint16_t flags, uint8_t status);
    bool sendRequests(uint16_t flags);
    bool handleMessage(const uint8_t* buffer, size_t length);

    
    // Thread management
    vex::thread* read_thread;
    vex::thread* write_thread;
    vex::timer timer;
    bool running;
    
    // Mutexes for thread safety
    mutable vex::mutex data_mutex;
    vex::mutex state_mutex;

    std::queue<uint16_t> pending_requests;              // Queue for storing requests to jetson
    std::queue<uint16_t> pending_acknowledgments;

    bool request_in_progress ;
    uint8_t request_retry_count;
    
    // Communication state
    uint16_t request_flags;                 // Current Brain request flags
    uint16_t response_flags;                // Current Jetson request flags
    uint32_t last_send_time;                // Timestamp of last data send
    uint32_t last_received_time;            // Timestamp of last received data
    uint32_t last_request_time;             // Timestamp of last request data
    
    
    // Data storage (to be connected to actual sensors/systems)
    MotorCommand current_motor_cmd;
    ControlFlags current_control_flags;
    uint32_t JetsonBatteryLvl;
    // Statistics tracking
    StatsManager stats;
};

} // namespace Jetson

#endif // JETSON_COMM_H_