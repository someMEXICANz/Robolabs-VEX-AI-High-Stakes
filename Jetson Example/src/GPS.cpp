#include <GPS.h>

GPS::GPS(boost::asio::io_service& service, const std::string& new_port)
    : 
      port(new_port),
      io_service(service),
      running(false),
      connected(false),
      current_position{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, std::chrono::system_clock::now()}
{
    
    if (!port.empty()) {
        initializePort();
    }
}

GPS::~GPS() 
{
    stop();
}

bool GPS::start() {
    if(running)
    {
        std::cerr << "GPS read thread is already running" << std::endl; 
        return true;
    }
    
    if (!connected && !reconnect()) 
    {
        return false;
    }
    try{  
       
        running = true;
        read_thread = std::make_unique<std::thread>(&GPS::readLoop, this);
        return true;
        
    } catch (const std::exception& e) 
    {
        std::cerr << "Failed to start GPS read threads: " << e.what() << std::endl;
        running = false;
        return false;  
    }
}

void GPS::stop() {
    running = false;
    
    if (read_thread && read_thread->joinable()) {
        read_thread->join();
    }
    
    read_thread.reset();
    
    if (serial_port && serial_port->is_open()) {
        serial_port->close();
    }
    
    connected = false;
}

bool GPS::restart() {
    stop();
    return start();
}

bool GPS::reconnect() {
    if (serial_port && serial_port->is_open()) {
        serial_port->close();
    }
    return initializePort();
}



bool GPS::initializePort() {
    try {
        serial_port = std::make_unique<boost::asio::serial_port>(io_service);
        serial_port->open(port);
        serial_port->set_option(boost::asio::serial_port_base::baud_rate(115200));
        serial_port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_port->set_option(boost::asio::serial_port_base::character_size(8));
        serial_port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        serial_port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        
        connected = true;
        std::cerr << "GPS initialized on port " << port << std::endl;
        return true;
    } catch (const boost::system::system_error& e) {
        std::cerr << "Failed to initialize GPS port: " << e.what() << std::endl;
        connected = false;
        return false;
    }
}

void GPS::readLoop() {
    std::cerr << "GPS read loop started" << std::endl;
    boost::asio::deadline_timer timer(io_service);
    
    while (running) {
        try {
            if (!connected && !reconnect()) {
                std::this_thread::sleep_for(RECONNECT_DELAY);
                continue;
            }

            std::vector<unsigned char> buffer(16); // Based on Python code packet size
            boost::system::error_code ec;
            
            // Set timeout
            timer.expires_from_now(boost::posix_time::milliseconds(READ_TIMEOUT.count()));
            
            // Read until end marker 0xCC33 (based on Python code)
            std::size_t bytes_read = boost::asio::read(*serial_port, boost::asio::buffer(buffer), 
                                     boost::asio::transfer_at_least(16), ec);
            
            if (ec) {
                std::cerr << "GPS read error: " << ec.message() << std::endl;
                connected = false;
                continue;
            }

            if (bytes_read == 16 && buffer[14] == 0xCC && buffer[15] == 0x33) {
                processBuffer(buffer);
            }
        } catch (const std::exception& e) {
            std::cerr << "Error in GPS read loop: " << e.what() << std::endl;
            connected = false;
            std::this_thread::sleep_for(RECONNECT_DELAY);
        }
    }
    
    std::cerr << "GPS read loop stopped" << std::endl;
}

void GPS::processBuffer(const std::vector<unsigned char>& buffer) {
    // Extract status byte
    uint32_t current_status = buffer[1];
    float quality = calculateGPSQuality(current_status);

    // Parse position data (following Python implementation)
    int16_t x_raw, y_raw, z_raw, az_raw, el_raw, rot_raw;
    std::memcpy(&x_raw, &buffer[2], 2);
    std::memcpy(&y_raw, &buffer[4], 2);
    std::memcpy(&z_raw, &buffer[6], 2);
    std::memcpy(&az_raw, &buffer[8], 2);
    std::memcpy(&el_raw, &buffer[10], 2);
    std::memcpy(&rot_raw, &buffer[12], 2);

    // Update position
    std::lock_guard<std::mutex> lock(position_mutex);
    // Convert to proper units, just like in Python code
    current_position.x = x_raw / 10000.0f;
    current_position.y = y_raw / 10000.0f;
    current_position.z = z_raw / 10000.0f;
    current_position.azimuth = az_raw / 32768.0f * 180.0f; 
    current_position.elevation = el_raw / 32768.0f * 180.0f;
    current_position.rotation = rot_raw / 32768.0f * 180.0f;
   
}

GPSPosition GPS::getGPSposition() const {
    std::lock_guard<std::mutex> lock(position_mutex);
    return current_position;
}




float GPS::calculateGPSQuality(uint32_t status) const 
{
    // Start with maximum quality
    float quality = 1.0f;
    
    // Critical errors - position is unreliable
    if (!(status & GPS::STATUS_CONNECTED)) 
    {
        return 0.0f;  // Very low quality
    }
    
    // Major data acquisition issues
    if ((status & GPS::STATUS_NODOTS) || (status & GPS::STATUS_NOBITS)  || 
        (status & GPS::STATUS_NOSOLUTION)) 
    {
        quality *= 0.1f;
    }
    
    // Data processing issues
    if ((status & GPS::STATUS_NORAWBITS) || (status & GPS::STATUS_NOGROUPS) || 
        (status & GPS::STATUS_PIXELERROR)) {
        quality *= 0.5f;
    }
    
    // Position estimation or jumps
    if ((status & GPS::STATUS_ANGLEJUMP) || (status & GPS::STATUS_POSJUMP)) {
        quality *= 0.7f;
    }
    
    // Minor issues
    if ((status & GPS::STATUS_SOLVER) || (status & GPS::STATUS_KALMAN_EST)) {
        quality *= 0.8f;
    }
    
    return quality;
}