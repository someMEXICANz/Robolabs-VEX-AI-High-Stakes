#include "Position.h"

Position::Position() 
    : x(0), y(0), z(0),
      confidence(0.0f), timestamp(std::chrono::system_clock::now()) {}

Position::Position(float x, float y, float z, float azimuth, float elevation, float rotation)
    : x(x), y(y), z(z), 
      confidence(1.0f), timestamp(std::chrono::system_clock::now()) {}

void Position::printPosition() const {
    std::cout << "Position (x, y, z): (" << std::fixed << std::setprecision(3) 
              << x << ", " << y << ", " << z << ") meters\n"
              << "Confidence: " << std::setprecision(2) << confidence << std::endl;
}

float Position::distanceTo(const Position& other) const {
    float dx = x - other.x;
    float dy = y - other.y;
    float dz = z - other.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

float Position::distanceTo2D(const Position& other) const {
    float dx = x - other.x;
    float dy = y - other.y;
    return std::sqrt(dx*dx + dy*dy);
}