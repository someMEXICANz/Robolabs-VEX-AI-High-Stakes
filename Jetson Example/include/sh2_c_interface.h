// sh2_c_interface.h
#ifndef SH2_C_INTERFACE_H
#define SH2_C_INTERFACE_H

/**
 * @file sh2_c_interface.h
 * @brief C/C++ interface bridge for SH2 sensor hub library
 * 
 * This header provides a clean way to include the SH2 C library
 * in C++ code without having to write extern "C" everywhere.
 * 
 * Simply include this header instead of the individual SH2 headers.
 */

#ifdef __cplusplus
extern "C" {
#endif

// Core SH2 library headers
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "sh2_hal.h"
#include "sh2_util.h"
#include "euler.h"
#include "shtp.h"

#ifdef __cplusplus
}
#endif

// C++ convenience additions
#ifdef __cplusplus

#include <string>
#include <iostream>
#include <math.h> 


/**
 * @brief Convert SH2 error code to string
 * @param error SH2 error code
 * @return Human-readable error description
 */
inline std::string sh2ErrorToString(int error) {
    switch (error) {
        case SH2_OK:                 return "Success";
        case SH2_ERR:                return "General Error";
        case SH2_ERR_BAD_PARAM:      return "Bad Parameter";
        case SH2_ERR_OP_IN_PROGRESS: return "Operation In Progress";
        case SH2_ERR_IO:             return "I/O Error";
        case SH2_ERR_HUB:            return "Hub Error";
        case SH2_ERR_TIMEOUT:        return "Timeout";
        default:                     return "Unknown Error (" + std::to_string(error) + ")";
    }
}

/**
 * @brief Convert sensor ID to string
 * @param sensorId SH2 sensor ID
 * @return Human-readable sensor name
 */
inline std::string sensorIdToString(sh2_SensorId_t sensorId) {
    switch (sensorId) {
        case SH2_RAW_ACCELEROMETER:           return "Raw Accelerometer";
        case SH2_ACCELEROMETER:               return "Accelerometer";
        case SH2_LINEAR_ACCELERATION:         return "Linear Acceleration";
        case SH2_GRAVITY:                     return "Gravity";
        case SH2_RAW_GYROSCOPE:               return "Raw Gyroscope";
        case SH2_GYROSCOPE_CALIBRATED:        return "Gyroscope (Calibrated)";
        case SH2_GYROSCOPE_UNCALIBRATED:      return "Gyroscope (Uncalibrated)";
        case SH2_RAW_MAGNETOMETER:            return "Raw Magnetometer";
        case SH2_MAGNETIC_FIELD_CALIBRATED:   return "Magnetic Field (Calibrated)";
        case SH2_MAGNETIC_FIELD_UNCALIBRATED: return "Magnetic Field (Uncalibrated)";
        case SH2_ROTATION_VECTOR:             return "Rotation Vector";
        case SH2_GAME_ROTATION_VECTOR:        return "Game Rotation Vector";
        case SH2_GEOMAGNETIC_ROTATION_VECTOR: return "Geomagnetic Rotation Vector";
        case SH2_PRESSURE:                    return "Pressure";
        case SH2_AMBIENT_LIGHT:               return "Ambient Light";
        case SH2_HUMIDITY:                    return "Humidity";
        case SH2_PROXIMITY:                   return "Proximity";
        case SH2_TEMPERATURE:                 return "Temperature";
        case SH2_TAP_DETECTOR:                return "Tap Detector";
        case SH2_STEP_DETECTOR:               return "Step Detector";
        case SH2_STEP_COUNTER:                return "Step Counter";
        case SH2_SIGNIFICANT_MOTION:          return "Significant Motion";
        case SH2_STABILITY_CLASSIFIER:        return "Stability Classifier";
        case SH2_SHAKE_DETECTOR:              return "Shake Detector";
        case SH2_FLIP_DETECTOR:               return "Flip Detector";
        case SH2_PICKUP_DETECTOR:             return "Pickup Detector";
        case SH2_STABILITY_DETECTOR:          return "Stability Detector";
        case SH2_PERSONAL_ACTIVITY_CLASSIFIER: return "Personal Activity Classifier";
        case SH2_SLEEP_DETECTOR:              return "Sleep Detector";
        case SH2_TILT_DETECTOR:               return "Tilt Detector";
        case SH2_POCKET_DETECTOR:             return "Pocket Detector";
        case SH2_CIRCLE_DETECTOR:             return "Circle Detector";
        case SH2_HEART_RATE_MONITOR:          return "Heart Rate Monitor";
        case SH2_ARVR_STABILIZED_RV:          return "AR/VR Stabilized RV";
        case SH2_ARVR_STABILIZED_GRV:         return "AR/VR Stabilized GRV";
        case SH2_GYRO_INTEGRATED_RV:          return "Gyro Integrated RV";
        case SH2_IZRO_MOTION_REQUEST:         return "iZRO Motion Request";
        case SH2_RAW_OPTICAL_FLOW:            return "Raw Optical Flow";
        case SH2_DEAD_RECKONING_POSE:         return "Dead Reckoning Pose";
        case SH2_WHEEL_ENCODER:               return "Wheel Encoder";
        default:                              return "Unknown Sensor (0x" + 
                                                     std::to_string(sensorId) + ")";
    }
}

/**
 * @brief Print sensor value in a human-readable format
 * @param value Decoded sensor value
 */
inline void printSensorValue(const sh2_SensorValue_t& value) {
    std::cout << "[" << sensorIdToString(value.sensorId) << "] ";
    
    switch (value.sensorId) {
        case SH2_ACCELEROMETER:
            std::cout << "X=" << value.un.accelerometer.x 
                      << " Y=" << value.un.accelerometer.y 
                      << " Z=" << value.un.accelerometer.z << " m/s²";
            break;
            
        case SH2_GYROSCOPE_CALIBRATED:
            std::cout << "X=" << value.un.gyroscope.x 
                      << " Y=" << value.un.gyroscope.y 
                      << " Z=" << value.un.gyroscope.z << " rad/s";
            break;
            
        case SH2_MAGNETIC_FIELD_CALIBRATED:
            std::cout << "X=" << value.un.magneticField.x 
                      << " Y=" << value.un.magneticField.y 
                      << " Z=" << value.un.magneticField.z << " µT";
            break;
            
        case SH2_ROTATION_VECTOR:
            {
                float yaw, pitch, roll;
                q_to_ypr(value.un.rotationVector.real,
                         value.un.rotationVector.i,
                         value.un.rotationVector.j,
                         value.un.rotationVector.k,
                         &yaw, &pitch, &roll);
                
                std::cout << "Yaw=" << (yaw * 180.0f / M_PI) 
                          << "° Pitch=" << (pitch * 180.0f / M_PI)
                          << "° Roll=" << (roll * 180.0f / M_PI) << "°";
            }
            break;
            
        default:
            std::cout << "Data available";
            break;
    }
    
    std::cout << " (Status=" << static_cast<int>(value.status) << ")" << std::endl;
}

#endif // __cplusplus

#endif // SH2_C_INTERFACE_H