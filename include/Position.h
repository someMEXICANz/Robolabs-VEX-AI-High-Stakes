#ifndef POSITION_H
#define POSITION_H

#include <iostream>
#include <cmath>
#include <chrono>
#include <iomanip>

class Position {
public:
    // Core position data
    float x, y, z;
    float azimuth, elevation, rotation;
    std::chrono::system_clock::time_point timestamp;  // Use system_clock consistently
    float confidence;  // Added confidence level (0.0-1.0)
   
    // Constructors
    Position();
    Position(float x, float y, float z, float azimuth = 0.0f, float elevation = 0.0f, float rotation = 0.0f);
    
    // Utility methods
    void printPosition() const;
    float distanceTo(const Position& other) const;
    float distanceTo2D(const Position& other) const;  // 2D distance (ignores z)
    
    // Helper methods for path planning
    float getHeading() const { return azimuth; }  // Alias for clarity
    
    // Create a 2D representation (for path planning)
    void to2D(float& out_x, float& out_y, float& out_heading) const {
        out_x = x;
        out_y = y;
        out_heading = azimuth;
    }
};

#endif // POSITION_H