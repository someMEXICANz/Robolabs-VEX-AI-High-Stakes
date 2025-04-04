#ifndef BRAIN_COMM_STATS_H
#define BRAIN_COMM_STATS_H

#include "BrainCommTypes.h"
#include <deque>
#include <mutex>

namespace Brain {

// Statistics tracking
struct TransmitStats {
    std::atomic<size_t> total_transmissions{0};
    std::atomic<size_t> failed_transmissions{0};
    std::atomic<size_t> missed_deadlines{0};
    std::atomic<size_t> write_buffer_overruns{0};
    std::chrono::microseconds max_transmission_delay{0};
    std::chrono::microseconds avg_transmission_delay{0};
};

struct ReceiveStats {
    std::atomic<size_t> total_reads{0};
    std::atomic<size_t> failed_reads{0};
    std::atomic<size_t> invalid_requests{0};
    std::atomic<size_t> lost_connections{0};
    std::atomic<size_t> reestablished_connections{0};
};




class StatsManager {
public:
    StatsManager();

    // Error logging
    void logError(CommError error, const std::string& description);
    std::vector<ErrorLog> getRecentErrors(size_t count) const;
    void clearErrorLog();

    // Statistics
    const TransmitStats& getTransmitStats() const { return tx_stats; }
    TransmitStats& getTransmitStats()  { return tx_stats; }
    const ReceiveStats& getReceiveStats() const { return rx_stats; }
    void clearStats();
    
    // Update methods
    void updateTimingStats(std::chrono::microseconds transmission_time);
    void incrementTransmitStats(bool success);
    void incrementReceiveStats(bool success);
    void recordDeadlineMiss();
    void recordBufferOverrun();
    void recordConnectionStatus(bool connected);

    // Health check
    bool isHealthy() const;

private:
    TransmitStats tx_stats;
    ReceiveStats rx_stats;
    std::deque<ErrorLog> error_log;
    mutable std::mutex stats_mutex;
};

} // namespace brain_comm

#endif




