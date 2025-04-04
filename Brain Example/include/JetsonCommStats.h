#ifndef JETSON_COMM_STATS_H_
#define JETSON_COMM_STATS_H_

#include "JetsonCommTypes.h"

namespace Jetson {

// Statistics tracking
struct TransmitStats {
    // Core counters
    uint32_t total_packets;             // Total packets sent
    uint32_t failed_packets;            // Failed transmissions
    uint32_t total_bytes;               // Total bytes transmitted
    
    // Data rate tracking
    uint32_t last_byte_count;           // For rate calculation 
    uint32_t last_packet_count;         // For packet rate calculation
    uint32_t last_rate_update_time;     // When rate was last calculated
    float bytes_per_second;             // Current transmit rate
    uint32_t packets_per_second;        // Packet rate
};

struct ReceiveStats {
    // Core counters
    uint32_t total_packets;             // Total packets received
    uint32_t failed_packets;            // Failed receptions
    uint32_t total_bytes;               // Total bytes received
    
    // Data rate tracking
    uint32_t last_byte_count;           // For rate calculation
    uint32_t last_packet_count;         // For packet rate calculation
    uint32_t last_rate_update_time;     // When rate was last calculated
    float bytes_per_second;             // Current receive rate
    uint32_t packets_per_second;        // Packet rate
    
    // Protocol-specific counts
    uint32_t request_timeouts;          // Request-specific timeouts
    uint32_t buffer_overruns;           // Read buffer overflows
};

class StatsManager {
public:
    StatsManager();

    // Error logging
    void logError(CommError error, const char* description);
    const ErrorLog* getRecentErrors(size_t& count) const;
    void clearErrorLog();


    // Statistics access
    const TransmitStats& getTransmitStats() const { return tx_stats_; }
    const ReceiveStats& getReceiveStats() const { return rx_stats_; }
    void clearStats();
    
    // Primary recording methods
    void recordTransmission(bool success, size_t bytes);
    void recordReception(bool success, size_t bytes);
    void recordBufferOverrun();
    void recordRequestTimeout();
    
    // Rate calculation
    void updateRates() ;
    
    // Health check and metrics
    bool isHealthy() const;
    float getTransmitSuccessRate() const;
    float getReceiveSuccessRate() const;
    float getConnectionQualityScore() const;
    
    // Data rate access
    float getTxBytesPerSecond() const;
    float getRxBytesPerSecond() const;
    uint32_t getTxPacketsPerSecond() const;
    uint32_t getRxPacketsPerSecond() const;
    
    // Periodic reporting
    void logStatsSummary(FILE* log_file) const;

private:
    TransmitStats tx_stats_;
    ReceiveStats rx_stats_;
    ErrorLog error_log_[CommConstants::MAX_ERROR_LOG];
    size_t error_log_count_;
    mutable vex::mutex stats_mutex;
};

} // namespace Jetson

#endif // JETSON_COMM_STATS_H_