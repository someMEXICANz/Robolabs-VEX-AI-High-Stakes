#include "vex.h"
#include "robot-config.h"

using namespace vex;

// Display communication statistics
// Display communication statistics
static void dashboardComm(int ox, int oy, int width, int height) {
    static uint32_t update_time = 0;
    color grey = vex::color(0x404040);

    // Set up display region
    Brain.Screen.setClipRegion(ox, oy, width, height);
    Brain.Screen.setFont(mono15);

    // Draw border and title bar
    Brain.Screen.setPenColor(yellow);
    Brain.Screen.drawRectangle(ox, oy, width, height, black);
    Brain.Screen.drawRectangle(ox, oy, width, 20, grey);

    // Title
    Brain.Screen.setPenColor(yellow);
    Brain.Screen.setFillColor(grey);
    Brain.Screen.printAt(ox + 10, oy + 15, "Communication Statistics");
    oy += 25;

    // Column headers
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFillColor(black);
    Brain.Screen.printAt(ox + 95, oy + 15, "Read     Write");
    
    const auto& tx_stats = jetson.getStats().getTransmitStats();
    const auto& rx_stats = jetson.getStats().getReceiveStats();
    
    // Stats rows
    Brain.Screen.printAt(ox + 10, oy + 40, "Success:");
    Brain.Screen.printAt(ox + 95, oy + 40, "%.1f%%     %.1f%%", 
                        jetson.getStats().getReceiveSuccessRate() * 100.0f,
                        jetson.getStats().getTransmitSuccessRate() * 100.0f);

    Brain.Screen.printAt(ox + 10, oy + 55, "Errors:");
    Brain.Screen.printAt(ox + 95, oy + 55, "%d        %d", 
                        rx_stats.failed_packets,
                        tx_stats.failed_packets);

    Brain.Screen.printAt(ox + 10, oy + 70, "B/sec:");
    Brain.Screen.printAt(ox + 95, oy + 70, "%.1f     %.1f",
                        jetson.getStats().getRxBytesPerSecond(), 
                        jetson.getStats().getTxBytesPerSecond());

    Brain.Screen.printAt(ox + 10, oy + 85, "Pkt/sec:");
    Brain.Screen.printAt(ox + 95, oy + 85, "%d        %d",
                        jetson.getStats().getRxPacketsPerSecond(),
                        jetson.getStats().getTxPacketsPerSecond());
                        
    // Quality score
    Brain.Screen.printAt(ox + 10, oy + 100, "Quality:");
    
    float quality = jetson.getStats().getConnectionQualityScore();
    // Color-code quality score
    if (quality > 0.9) {
        Brain.Screen.setPenColor(green);
    } else if (quality > 0.7) {
        Brain.Screen.setPenColor(yellow);
    } else {
        Brain.Screen.setPenColor(red);
    }
    
    Brain.Screen.printAt(ox + 95, oy + 100, "%.1f%%", quality * 100.0f);
    Brain.Screen.setPenColor(white); // Reset color
    
    // Force a stats update
    if (Brain.Timer.system() > update_time) {
        update_time = Brain.Timer.system() + 1000; // Update once per second
        jetson.getStats().updateRates();  // Make sure rates are current
    }
}

// Display position data
static void dashboardPosition(int ox, int oy, int width, int height) {
    color grey = vex::color(0x404040);

    // Set up display region
    Brain.Screen.setClipRegion(ox, oy, width, height);
    Brain.Screen.setFont(mono15);

    // Draw border and title bar
    Brain.Screen.setPenColor(yellow);
    Brain.Screen.drawRectangle(ox, oy, width, height, black);
    Brain.Screen.drawRectangle(ox, oy, width, 20, grey);

    // Title
    Brain.Screen.setPenColor(yellow);
    Brain.Screen.setFillColor(grey);
    Brain.Screen.printAt(ox + 10, oy + 15, "GPS Position Data");
    oy += 25;

    // Column headers
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFillColor(black);
    Brain.Screen.printAt(ox + 75, oy + 15, "X(mm)  Y(mm)  H(deg)");

    // Get sister position
    double sister_x, sister_y, sister_heading;
    link.get_remote_location(sister_x, sister_y, sister_heading);

    // Position rows
    Brain.Screen.printAt(ox + 10, oy + 40, "Left:");
    Brain.Screen.printAt(ox + 80, oy + 40, "%.2f   %.2f   %.2f",
                        Left_GPS.xPosition(mm),
                        Left_GPS.yPosition(mm),
                        Left_GPS.heading(degrees));

    Brain.Screen.printAt(ox + 10, oy + 60, "Right:");
    Brain.Screen.printAt(ox + 80, oy + 60, "%.2f   %.2f   %.2f",
                        Right_GPS.xPosition(mm),
                        Right_GPS.yPosition(mm),
                        Right_GPS.heading(degrees));

    Brain.Screen.printAt(ox + 10, oy + 80, "Sister:");
    Brain.Screen.printAt(ox + 80, oy + 80, "%.2f   %.2f   %.2f",
                        sister_x, sister_y, sister_heading);
}

// Display status and commands
static void dashboardStatus(int ox, int oy, int width, int height) {
    color grey = vex::color(0x404040);

    // Set up display region
    Brain.Screen.setClipRegion(ox, oy, width, height);
    Brain.Screen.setFont(mono15);

    // Draw border and title bar
    Brain.Screen.setPenColor(yellow);
    Brain.Screen.drawRectangle(ox, oy, width, height, black);
    Brain.Screen.drawRectangle(ox, oy, width, 20, grey);

    // Title
    Brain.Screen.setPenColor(yellow);
    Brain.Screen.setFillColor(grey);
    Brain.Screen.printAt(ox + 10, oy + 15, "Communication Data");
    oy += 25;

    // Get current commands
    MotorCommand motor_cmd = jetson.getMotorCommand();
    ControlFlags control_flags = jetson.getControlFlags();

    // Display flags
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFillColor(black);
    Brain.Screen.printAt(ox + 10, oy + 25, "Jetson Requests: 0x%04X", jetson.getCurrentResponse());
    Brain.Screen.printAt(ox + 10, oy + 40, "Brain Requests: 0x%04X", jetson.getCurrentRequests());
    
    // Display commands
    Brain.Screen.printAt(ox + 10, oy + 60, "Motor L: %.2f  R: %.2f",
                        motor_cmd.left_voltage, motor_cmd.right_voltage);
    Brain.Screen.printAt(ox + 10, oy + 75, "Macro: 0x%08X",
                        control_flags.macro_bits);
                        
    // Add battery information
    Brain.Screen.printAt(ox + 10, oy + 95, "Battery: Brain: %d%%  Jetson: %d%%",
                        Brain.Battery.capacity(vex::percentUnits::pct),
                        jetson.getJetsonBattery());
}

// Main dashboard task
int dashboardTask() {
    while(true) {
        // Draw top sections
        dashboardComm(0, 0, 240, 240);
        dashboardPosition(240, 0, 240, 120);
        
        // Draw bottom section
        dashboardStatus(0, 120, 480, 120);

        // Update at 30Hz
        Brain.Screen.render();
        this_thread::sleep_for(16);
    }
    return 0;
}