// BNO085.h
#ifndef BNO085_H
#define BNO085_H

#include "sh2_c_interface.h"
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <thread>
#include <atomic>

/**
 * @brief High-level C++ interface for BNO085 9-DOF sensor
 * 
 * This class provides an easy-to-use interface for the BNO085 sensor,
 * wrapping the SH2 library and handling all the low-level details.
 * 
 * Features:
 * - Simple initialization and configuration
 * - Convenient data reading methods
 * - Automatic sensor fusion and calibration
 * - Thread-safe operation
 * - Event-driven callbacks
 * - Error handling and diagnostics
 */
class BNO085 {
public:
    // ==========================================================================
    // DATA STRUCTURES
    // ==========================================================================
    
    /**
     * @brief 3D vector data (acceleration, angular velocity, magnetic field)
     */
    struct Vector3 {
        float x, y, z;
        
        Vector3() : x(0), y(0), z(0) {}
        Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
        
        // Utility methods
        float magnitude() const;
        Vector3 normalized() const;
        std::string toString() const;
    };
    
    /**
     * @brief Quaternion orientation data
     */
    struct Quaternion {
        float w, x, y, z;
        float accuracy;  // Accuracy estimate in radians (for rotation vector)
        
        Quaternion() : w(1), x(0), y(0), z(0), accuracy(0) {}
        Quaternion(float w, float x, float y, float z, float acc = 0) 
            : w(w), x(x), y(y), z(z), accuracy(acc) {}
        
        // Utility methods
        Vector3 toEulerAngles() const;  // Returns yaw, pitch, roll in degrees
        std::string toString() const;
    };
    
    /**
     * @brief Complete sensor reading with timestamp and status
     */
    struct SensorReading {
        uint64_t timestamp_us;
        uint8_t status;        // 0=unreliable, 1=low, 2=medium, 3=high accuracy
        uint8_t sequence;
        
        SensorReading() : timestamp_us(0), status(0), sequence(0) {}
    };
    
    /**
     * @brief Combined IMU data
     */
    struct IMUData : public SensorReading {
        Vector3 acceleration;      // m/s²
        Vector3 gyroscope;         // rad/s
        Vector3 magnetometer;      // µT
        
        IMUData() = default;
    };
    
    /**
     * @brief Orientation data
     */
    struct OrientationData : public SensorReading {
        Quaternion rotation;
        Vector3 euler;  // Yaw, pitch, roll in degrees (computed from quaternion)
        
        OrientationData() = default;
    };
    
    /**
     * @brief Sensor configuration
     */
    struct SensorConfig {
        uint32_t reportInterval_us;     // Report interval in microseconds
        bool enabled;                   // Whether sensor is enabled
        bool wakeupEnabled;             // Wake system on sensor event
        bool alwaysOnEnabled;           // Keep sensor on during sleep
        
        SensorConfig() : reportInterval_us(10000), enabled(false), 
                        wakeupEnabled(false), alwaysOnEnabled(false) {}
        
        // Convenience methods
        void setFrequency(float hz) { 
            reportInterval_us = static_cast<uint32_t>(1000000.0f / hz); 
        }
        float getFrequency() const { 
            return 1000000.0f / reportInterval_us; 
        }
    };
    
    // ==========================================================================
    // CALLBACK TYPES
    // ==========================================================================
    
    using IMUCallback = std::function<void(const IMUData& data)>;
    using OrientationCallback = std::function<void(const OrientationData& data)>;
    using SensorEventCallback = std::function<void(sh2_SensorId_t sensorId, const sh2_SensorValue_t& value)>;
    using ErrorCallback = std::function<void(const std::string& error)>;
    
    // ==========================================================================
    // ENUMS
    // ==========================================================================
    
    enum class CalibrationStatus {
        UNCALIBRATED = 0,
        LOW_ACCURACY = 1,
        MEDIUM_ACCURACY = 2,
        HIGH_ACCURACY = 3
    };
    
    enum class SensorType {
        ACCELEROMETER,
        GYROSCOPE, 
        MAGNETOMETER,
        ROTATION_VECTOR,
        GAME_ROTATION_VECTOR,
        LINEAR_ACCELERATION,
        GRAVITY
    };
    
    // ==========================================================================
    // CONSTRUCTOR/DESTRUCTOR
    // ==========================================================================
    
    /**
     * @brief Create BNO085 instance with default I2C settings
     */
    BNO085();
    
    /**
     * @brief Create BNO085 instance with custom I2C settings
     * @param i2c_bus I2C bus path (e.g., "/dev/i2c-1")
     * @param i2c_address I2C device address (e.g., 0x4A)
     */
    BNO085(const std::string& i2c_bus, uint8_t i2c_address);
    
    /**
     * @brief Destructor - automatically shuts down sensor
     */
    ~BNO085();
    
    // Prevent copying
    BNO085(const BNO085&) = delete;
    BNO085& operator=(const BNO085&) = delete;
    
    // ==========================================================================
    // INITIALIZATION AND CONTROL
    // ==========================================================================
    
    /**
     * @brief Initialize the sensor
     * @return true on success, false on failure
     */
    bool initialize();
    
    /**
     * @brief Shutdown the sensor and clean up resources
     */
    void shutdown();
    
    /**
     * @brief Check if sensor is initialized and ready
     * @return true if ready, false otherwise
     */
    bool isInitialized() const { return initialized_; }
    
    /**
     * @brief Start the service thread for automatic data processing
     * Call this after initialization to begin receiving sensor data
     */
    void startService();
    
    /**
     * @brief Stop the service thread
     */
    void stopService();
    
    /**
     * @brief Check if service thread is running
     */
    bool isServiceRunning() const { return service_running_; }
    
    // ==========================================================================
    // SENSOR CONFIGURATION
    // ==========================================================================
    
    /**
     * @brief Enable a sensor with specific configuration
     * @param sensor Sensor type to enable
     * @param config Configuration settings
     * @return true on success, false on failure
     */
    bool enableSensor(SensorType sensor, const SensorConfig& config);
    
    /**
     * @brief Disable a sensor
     * @param sensor Sensor type to disable
     * @return true on success, false on failure
     */
    bool disableSensor(SensorType sensor);
    
    /**
     * @brief Enable basic IMU sensors with default settings
     * Enables accelerometer, gyroscope, and magnetometer at 100Hz
     * @return true on success, false on failure
     */
    bool enableBasicIMU();
    
    /**
     * @brief Enable orientation sensing with default settings
     * Enables rotation vector at 50Hz
     * @return true on success, false on failure
     */
    bool enableOrientation();
    
    /**
     * @brief Get current configuration for a sensor
     * @param sensor Sensor type
     * @param config Output configuration
     * @return true on success, false on failure
     */
    bool getSensorConfig(SensorType sensor, SensorConfig& config);
    
    // ==========================================================================
    // DATA READING
    // ==========================================================================
    
    /**
     * @brief Get the latest acceleration data
     * @param data Output acceleration data
     * @return true if data is available and valid
     */
    bool getAcceleration(Vector3& data);
    
    /**
     * @brief Get the latest gyroscope data
     * @param data Output angular velocity data
     * @return true if data is available and valid
     */
    bool getGyroscope(Vector3& data);
    
    /**
     * @brief Get the latest magnetometer data
     * @param data Output magnetic field data
     * @return true if data is available and valid
     */
    bool getMagnetometer(Vector3& data);
    
    /**
     * @brief Get the latest orientation as quaternion
     * @param data Output quaternion data
     * @return true if data is available and valid
     */
    bool getOrientation(Quaternion& data);
    
    /**
     * @brief Get the latest orientation as Euler angles
     * @param yaw Output yaw angle in degrees
     * @param pitch Output pitch angle in degrees  
     * @param roll Output roll angle in degrees
     * @return true if data is available and valid
     */
    bool getEulerAngles(float& yaw, float& pitch, float& roll);
    
    /**
     * @brief Get combined IMU data
     * @param data Output IMU data structure
     * @return true if all data is available and valid
     */
    bool getIMUData(IMUData& data);
    
    /**
     * @brief Get orientation data
     * @param data Output orientation data structure
     * @return true if data is available and valid
     */
    bool getOrientationData(OrientationData& data);
    
    // ==========================================================================
    // CALIBRATION
    // ==========================================================================
    
    /**
     * @brief Get calibration status for each sensor
     * @param accel Accelerometer calibration status
     * @param gyro Gyroscope calibration status
     * @param mag Magnetometer calibration status
     * @param system Overall system calibration status
     */
    void getCalibrationStatus(CalibrationStatus& accel, CalibrationStatus& gyro, 
                             CalibrationStatus& mag, CalibrationStatus& system);
    
    /**
     * @brief Check if the sensor is fully calibrated
     * @return true if all sensors have high accuracy calibration
     */
    bool isFullyCalibrated();
    
    /**
     * @brief Start calibration procedure
     * @param interval_us Sensor report interval during calibration
     * @return true on success, false on failure
     */
    bool startCalibration(uint32_t interval_us = 10000);
    
    /**
     * @brief Finish calibration and save results
     * @return true on success, false on failure
     */
    bool finishCalibration();
    
    /**
     * @brief Save current dynamic calibration data to flash
     * @return true on success, false on failure
     */
    bool saveCalibration();
    
    // ==========================================================================
    // CALLBACKS
    // ==========================================================================
    
    /**
     * @brief Set callback for IMU data events
     * @param callback Function to call when new IMU data arrives
     */
    void setIMUCallback(IMUCallback callback) { imu_callback_ = callback; }
    
    /**
     * @brief Set callback for orientation data events
     * @param callback Function to call when new orientation data arrives
     */
    void setOrientationCallback(OrientationCallback callback) { orientation_callback_ = callback; }
    
    /**
     * @brief Set callback for raw sensor events
     * @param callback Function to call for any sensor event
     */
    void setSensorEventCallback(SensorEventCallback callback) { sensor_callback_ = callback; }
    
    /**
     * @brief Set callback for error events
     * @param callback Function to call when errors occur
     */
    void setErrorCallback(ErrorCallback callback) { error_callback_ = callback; }
    
    // ==========================================================================
    // DIAGNOSTICS AND STATUS
    // ==========================================================================
    
    /**
     * @brief Get product information from the sensor
     * @return Product information string
     */
    std::string getProductInfo();
    
    /**
     * @brief Get the last error message
     * @return Error string, empty if no error
     */
    std::string getLastError() const { return last_error_; }
    
    /**
     * @brief Check if sensor is responding
     * @return true if sensor responds to I2C communication
     */
    bool isConnected();
    
    /**
     * @brief Reset the sensor (software reset)
     * @return true on success, false on failure
     */
    bool reset();
    
    /**
     * @brief Get sensor metadata for a specific sensor
     * @param sensor Sensor type
     * @return Metadata string
     */
    std::string getSensorMetadata(SensorType sensor);

private:
    // ==========================================================================
    // PRIVATE MEMBERS
    // ==========================================================================
    
    // Hardware interface
    sh2_Hal_t* hal_;
    std::string i2c_bus_;
    uint8_t i2c_address_;
    
    // State management
    std::atomic<bool> initialized_;
    std::atomic<bool> service_running_;
    std::string last_error_;
    
    // Service thread
    std::unique_ptr<std::thread> service_thread_;
    
    // Data storage with thread-safe access
    mutable std::mutex data_mutex_;
    IMUData latest_imu_;
    OrientationData latest_orientation_;
    std::map<sh2_SensorId_t, sh2_SensorValue_t> latest_sensor_data_;
    
    // Callbacks
    IMUCallback imu_callback_;
    OrientationCallback orientation_callback_;
    SensorEventCallback sensor_callback_;
    ErrorCallback error_callback_;
    
    // ==========================================================================
    // PRIVATE METHODS
    // ==========================================================================
    
    void setError(const std::string& error);
    sh2_SensorId_t sensorTypeToId(SensorType sensor);
    void serviceLoop();
    
    // SH2 library callback handlers (static functions)
    static void eventCallbackWrapper(void* cookie, sh2_AsyncEvent_t* event);
    static void sensorCallbackWrapper(void* cookie, sh2_SensorEvent_t* event);
    
    // Instance callback handlers
    void handleAsyncEvent(sh2_AsyncEvent_t* event);
    void handleSensorEvent(sh2_SensorEvent_t* event);
    
    // Data processing
    void updateIMUData(const sh2_SensorValue_t& value);
    void updateOrientationData(const sh2_SensorValue_t& value);
};

#endif // BNO085_H