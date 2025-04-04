#include "vex.h"

namespace Jetson {

StatsManager::StatsManager() : error_log_count_(0) {
    clearStats();
}


void StatsManager::logError(CommError error, const char* description) {
    stats_mutex.lock();
    
    // Create new error log entry
    ErrorLog& log = error_log_[error_log_count_ % CommConstants::MAX_ERROR_LOG];
    log.error_type = error;
    strncpy(log.description, description, sizeof(log.description) - 1);
    log.description[sizeof(log.description) - 1] = '\0'; // Ensure null termination
    log.timestamp = vex::timer::system();
    
    if (error_log_count_ < CommConstants::MAX_ERROR_LOG) {
        error_log_count_++;
    }
    
    stats_mutex.unlock();
}

const ErrorLog* StatsManager::getRecentErrors(size_t& count) const {
    stats_mutex.lock();
    count = error_log_count_ < CommConstants::MAX_ERROR_LOG ? 
            error_log_count_ : CommConstants::MAX_ERROR_LOG;
    stats_mutex.unlock();
    return error_log_;
}

void StatsManager::clearErrorLog() {
    stats_mutex.lock();
    error_log_count_ = 0;
    stats_mutex.unlock();
}

void StatsManager::clearStats() {
    stats_mutex.lock();
    
    // Clear transmit stats
    tx_stats_.total_packets = 0;
    tx_stats_.failed_packets = 0;
    tx_stats_.total_bytes = 0;
    tx_stats_.last_byte_count = 0;
    tx_stats_.last_packet_count = 0;
    tx_stats_.last_rate_update_time = vex::timer::system();
    tx_stats_.bytes_per_second = 0.0f;
    tx_stats_.packets_per_second = 0;
    
    // Clear receive stats
    rx_stats_.total_packets = 0;
    rx_stats_.failed_packets = 0;
    rx_stats_.total_bytes = 0;
    rx_stats_.last_byte_count = 0;
    rx_stats_.last_packet_count = 0;
    rx_stats_.last_rate_update_time = vex::timer::system();
    rx_stats_.bytes_per_second = 0.0f;
    rx_stats_.packets_per_second = 0;
    rx_stats_.request_timeouts = 0;
    rx_stats_.buffer_overruns = 0;
    
    stats_mutex.unlock();
}

void StatsManager::recordTransmission(bool success, size_t bytes) {
    stats_mutex.lock();
    tx_stats_.total_packets++;
    tx_stats_.total_bytes += bytes;
    if (!success) {
        tx_stats_.failed_packets++;
    }
    stats_mutex.unlock();
    
    // Update rates periodically
    updateRates();
}

void StatsManager::recordReception(bool success, size_t bytes) {
    stats_mutex.lock();
    rx_stats_.total_packets++;
    rx_stats_.total_bytes += bytes;
    if (!success) {
        rx_stats_.failed_packets++;
    }
    stats_mutex.unlock();
    
    // Update rates periodically
    updateRates();
}

void StatsManager::recordBufferOverrun() {
    stats_mutex.lock();
    rx_stats_.buffer_overruns++;
    stats_mutex.unlock();
}

void StatsManager::recordRequestTimeout() {
    stats_mutex.lock();
    rx_stats_.request_timeouts++;
    stats_mutex.unlock();
}

void StatsManager::updateRates() {
    stats_mutex.lock();
    
    uint32_t current_time = vex::timer::system();
    
    // Update transmit rates (once per second)
    uint32_t tx_elapsed = current_time - tx_stats_.last_rate_update_time;
    if (tx_elapsed >= 1000) {
        // Calculate bytes per second
        tx_stats_.bytes_per_second = 
            (tx_stats_.total_bytes - tx_stats_.last_byte_count) * 1000.0f / tx_elapsed;
        
        // Calculate packets per second
        tx_stats_.packets_per_second = 
            (tx_stats_.total_packets - tx_stats_.last_packet_count) * 1000 / tx_elapsed;
        
        // Save values for next calculation
        tx_stats_.last_byte_count = tx_stats_.total_bytes;
        tx_stats_.last_packet_count = tx_stats_.total_packets;
        tx_stats_.last_rate_update_time = current_time;
    }
    
    // Update receive rates (once per second)
    uint32_t rx_elapsed = current_time - rx_stats_.last_rate_update_time;
    if (rx_elapsed >= 1000) {
        // Calculate bytes per second
        rx_stats_.bytes_per_second = 
            (rx_stats_.total_bytes - rx_stats_.last_byte_count) * 1000.0f / rx_elapsed;
        
        // Calculate packets per second
        rx_stats_.packets_per_second = 
            (rx_stats_.total_packets - rx_stats_.last_packet_count) * 1000 / rx_elapsed;
        
        // Save values for next calculation
        rx_stats_.last_byte_count = rx_stats_.total_bytes;
        rx_stats_.last_packet_count = rx_stats_.total_packets;
        rx_stats_.last_rate_update_time = current_time;
    }
    
    stats_mutex.unlock();
}

bool StatsManager::isHealthy() const {
    stats_mutex.lock();
    
    // Define health thresholds
    constexpr float MAX_FAILURE_RATE = 0.1f;  // 10% failure rate threshold
    
    // Calculate rates
    uint32_t total_tx = tx_stats_.total_packets;
    uint32_t total_rx = rx_stats_.total_packets;
    
    if (total_tx < 10 || total_rx < 10) {
        // Not enough data to determine health
        stats_mutex.unlock();
        return true;
    }
    
    float tx_failure_rate = static_cast<float>(tx_stats_.failed_packets) / total_tx;
    float rx_failure_rate = static_cast<float>(rx_stats_.failed_packets) / total_rx;
    
    stats_mutex.unlock();
    
    return tx_failure_rate < MAX_FAILURE_RATE && rx_failure_rate < MAX_FAILURE_RATE;
}

float StatsManager::getTransmitSuccessRate() const {
    stats_mutex.lock();
    float rate = tx_stats_.total_packets > 0 ?
                 1.0f - (static_cast<float>(tx_stats_.failed_packets) / tx_stats_.total_packets) :
                 0.0f;
    stats_mutex.unlock();
    return rate;
}

float StatsManager::getReceiveSuccessRate() const {
    stats_mutex.lock();
    float rate = rx_stats_.total_packets > 0 ?
                 1.0f - (static_cast<float>(rx_stats_.failed_packets) / rx_stats_.total_packets) :
                 0.0f;
    stats_mutex.unlock();
    return rate;
}

float StatsManager::getConnectionQualityScore() const {
    stats_mutex.lock();
    
    // Calculate weighted factors
    float tx_success_rate = getTransmitSuccessRate();
    float rx_success_rate = getReceiveSuccessRate();
    
    // Combine into overall score (0.0 to 1.0)
    float score = (tx_success_rate * 0.5f) + (rx_success_rate * 0.5f);
    
    stats_mutex.unlock();
    return score;
}

float StatsManager::getTxBytesPerSecond() const {
    stats_mutex.lock();
    float rate = tx_stats_.bytes_per_second;
    stats_mutex.unlock();
    return rate;
}

float StatsManager::getRxBytesPerSecond() const {
    stats_mutex.lock();
    float rate = rx_stats_.bytes_per_second;
    stats_mutex.unlock();
    return rate;
}

uint32_t StatsManager::getTxPacketsPerSecond() const {
    stats_mutex.lock();
    uint32_t rate = tx_stats_.packets_per_second;
    stats_mutex.unlock();
    return rate;
}

uint32_t StatsManager::getRxPacketsPerSecond() const {
    stats_mutex.lock();
    uint32_t rate = rx_stats_.packets_per_second;
    stats_mutex.unlock();
    return rate;
}

void StatsManager::logStatsSummary(FILE* log_file) const {
    stats_mutex.lock();
    
    fprintf(log_file, "----- COMM STATS SUMMARY -----\n");
    fprintf(log_file, "TX: %lu packets, %lu bytes (%.1f B/s, %lu pkt/s), %.1f%% success\n", 
            tx_stats_.total_packets,
            tx_stats_.total_bytes,
            tx_stats_.bytes_per_second,
            tx_stats_.packets_per_second,
            getTransmitSuccessRate() * 100.0f);
    
    fprintf(log_file, "RX: %lu packets, %lu bytes (%.1f B/s, %lu pkt/s), %.1f%% success\n", 
            rx_stats_.total_packets,
            rx_stats_.total_bytes,
            rx_stats_.bytes_per_second,
            rx_stats_.packets_per_second,
            getReceiveSuccessRate() * 100.0f);
    
    fprintf(log_file, "Buffer overruns: %lu, Request timeouts: %lu\n",
            rx_stats_.buffer_overruns,
            rx_stats_.request_timeouts);
    
    fprintf(log_file, "Connection Quality Score: %.2f\n", getConnectionQualityScore());
    
    fprintf(log_file, "Recent Errors: %zu\n", error_log_count_);
    
    stats_mutex.unlock();
}

} // namespace Jetson