#include "BrainCommStats.h"
#include <algorithm>

namespace Brain {

StatsManager::StatsManager() {
    clearStats();
    // Error log is already initialized as an empty deque
}

void StatsManager::logError(CommError error, const std::string& description) {
    std::lock_guard<std::mutex> lock(stats_mutex);
    
    ErrorLog log{
        error,
        description,
        std::chrono::system_clock::now()
    };
    
    error_log.push_back(log);
    
    // Keep error log size bounded
    if (error_log.size() > CommConstants::MAX_ERROR_LOG) {
        error_log.pop_front();
    }
}

std::vector<ErrorLog> StatsManager::getRecentErrors(size_t count) const {
    std::lock_guard<std::mutex> lock(stats_mutex);
    
    count = std::min(count, error_log.size());
    auto start = error_log.end() - count;
    
    return std::vector<ErrorLog>(start, error_log.end());
}

void StatsManager::clearErrorLog() {
    std::lock_guard<std::mutex> lock(stats_mutex);
    error_log.clear();
}

void StatsManager::clearStats() {
    tx_stats.total_transmissions = 0;
    tx_stats.failed_transmissions = 0;
    tx_stats.missed_deadlines = 0;
    tx_stats.write_buffer_overruns = 0;
    tx_stats.max_transmission_delay = std::chrono::microseconds(0);
    tx_stats.avg_transmission_delay = std::chrono::microseconds(0);
    
    rx_stats.total_reads = 0;
    rx_stats.failed_reads = 0;
    rx_stats.invalid_requests = 0;
    rx_stats.lost_connections = 0;
    rx_stats.reestablished_connections = 0;
}

void StatsManager::updateTimingStats(std::chrono::microseconds transmission_time) {
    std::lock_guard<std::mutex> lock(stats_mutex);
    
    // Update maximum delay
    if (transmission_time > tx_stats.max_transmission_delay) {
        tx_stats.max_transmission_delay = transmission_time;
    }
    
    // Update running average
    // Using exponential moving average with alpha = 0.1
    constexpr double alpha = 0.1;
    auto current_avg = tx_stats.avg_transmission_delay.count();
    auto new_avg = (alpha * transmission_time.count()) + 
                  ((1.0 - alpha) * current_avg);
    
    tx_stats.avg_transmission_delay = std::chrono::microseconds(
        static_cast<long long>(new_avg)
    );
}

void StatsManager::incrementTransmitStats(bool success) {
    tx_stats.total_transmissions++;
    if (!success) {
        tx_stats.failed_transmissions++;
    }
}

void StatsManager::incrementReceiveStats(bool success) {
    rx_stats.total_reads++;
    if (!success) {
        rx_stats.failed_reads++;
    }
}

void StatsManager::recordDeadlineMiss() {
    tx_stats.missed_deadlines++;
}

void StatsManager::recordBufferOverrun() {
    tx_stats.write_buffer_overruns++;
}

void StatsManager::recordConnectionStatus(bool connected) {
    if (!connected) {
        rx_stats.lost_connections++;
    } else {
        rx_stats.reestablished_connections++;
    }
}

bool StatsManager::isHealthy() const {
    std::lock_guard<std::mutex> lock(stats_mutex);
    
    // Define health thresholds
    constexpr double MAX_FAILURE_RATE = 0.1;  // 10% failure rate threshold
    constexpr double MAX_DEADLINE_MISS_RATE = 0.05;  // 5% deadline miss threshold
    
    // Calculate rates
    size_t total_tx = tx_stats.total_transmissions.load();
    size_t total_rx = rx_stats.total_reads.load();
    
    if (total_tx == 0 || total_rx == 0) return false;
    
    double tx_failure_rate = static_cast<double>(tx_stats.failed_transmissions) / total_tx;
    double rx_failure_rate = static_cast<double>(rx_stats.failed_reads) / total_rx;
    double deadline_miss_rate = static_cast<double>(tx_stats.missed_deadlines) / total_tx;
    
    return tx_failure_rate < MAX_FAILURE_RATE &&
           rx_failure_rate < MAX_FAILURE_RATE &&
           deadline_miss_rate < MAX_DEADLINE_MISS_RATE;
}

} // namespace brain_comm