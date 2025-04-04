#ifndef BRAIN_COMM_TYPES_H
#define BRAIN_COMM_TYPES_H

#include <atomic>
#include <chrono>
#include <string>
#include <vector>

namespace Brain {

// Request flags
enum class RequestFlag : uint16_t {
    NoData          = 0x0000,
    LeftGPSData     = 0x0001,
    RightGPSData    = 0x0002,
    SisterData      = 0x0004,
    MotorVoltages   = 0x0008,
    MacroControls   = 0x0010,
    BatteryLevel    = 0x0020,
};

struct Position2D {
    double x;
    double y;
    double heading;
}__attribute__((__packed__));

struct MotorCommand {
    float left_voltage;
    float right_voltage;
    uint32_t timestamp;
}__attribute__((__packed__));

struct ControlFlags {
    uint32_t macro_bits;
}__attribute__((__packed__));

// Error handling
enum class CommError {
    None = 0,
    ConnectionLost,
    WriteTimeout,
    ReadTimeout,
    InvalidRequest,
    InvalidPacket,
    BufferOverrun,
    TransmissionFailed,
    AcknowledgmentFailed,
    DeadlineMissed,
    RequestTimeout    
};

struct ErrorLog {
    CommError error_type;
    std::string description;
    std::chrono::system_clock::time_point timestamp;
};

// Communication constants
struct CommConstants {
    static constexpr size_t HEADER_SIZE = 4;
    static constexpr std::chrono::milliseconds READ_TIMEOUT{500};
    static constexpr std::chrono::milliseconds RETRY_DELAY{100};
    static constexpr std::chrono::milliseconds RESPONSE_UPDATE_PERIOD{20};  // 50Hz
    static constexpr std::chrono::milliseconds REQUEST_RETRY_PERIOD{500};  
    static constexpr std::chrono::milliseconds RECONNECT_DELAY{2500} ;
    static constexpr size_t MAX_REQUEST_RETRIES = 5;
    static constexpr size_t MAX_ERROR_LOG = 100;
    static constexpr size_t MAX_BUFFER_SIZE = 256;
    static constexpr uint8_t START_MARKER_1 = 0xAA;
    static constexpr uint8_t START_MARKER_2 = 0x55;
    static constexpr uint8_t END_MARKER_1 = 0x55;
    static constexpr uint8_t END_MARKER_2 = 0xAA;
};


enum class MessageType : uint8_t {
    NoOp = 0,           // No operation
    Request = 1,        // Request for data
    Response = 2,       // Response with data
    Acknowledgment = 3, // Acknowledgment of a request
    Error = 4,          // Error response
    Handshake = 5       // Initial handshake
};

// Updated header structure
struct RequestHeader {
    uint8_t  start_marker[2];  // Always 0xAA, 0x55
    uint8_t  message_type;     // MessageType value
    uint16_t flags;            // Request/response flags
    uint16_t length;           // Payload length
} __attribute__((__packed__));

// Add an end marker structure
struct EndMarker {
    uint8_t marker[2];  // Always 0x55, 0xAA (inverse of start)
} __attribute__((__packed__));


} // namespace brain_comm

#endif