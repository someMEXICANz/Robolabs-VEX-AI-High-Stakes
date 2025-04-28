#include "BrainComm.h"
#include "PortDetector.h"
#include <iomanip>
using namespace std;

namespace Brain {


// Constructor
BrainComm::BrainComm(boost::asio::io_service& service)
    : io_service(service)
    , port("")
    , running(false)
    , connected(false)
    , initialized(false)
    , started(false)
    , serial_port(nullptr)
    , request_flags(static_cast<uint16_t>(RequestFlag::NoData))
    , response_flags(static_cast<uint16_t>(RequestFlag::NoData))
    , request_in_progress(false)
    , request_retry_count(0)
    , last_send_time(0)
    , last_received_time(0)
    , last_request_time(0)
    , BrainBatteryLvl(0)
{
    // Initialize positions to zero values
    left_gps_position = {0.0, 0.0, 0.0};
    right_gps_position = {0.0, 0.0, 0.0};
    sister_position = {0.0, 0.0, 0.0};
    left_gps_offset = {0.0, 0.0, 0.0};
    right_gps_offset = {0.0, 0.0, 0.0};
    
    // Initialize command structures
    current_motor_command = {0.0, 0.0, 0};
    current_control_flags = {0};
    current_battery_lvl = 0;

    std::cerr << "Attempting to connect VEX Brain..." << std::endl;
    findPort();
    start();
}

BrainComm::~BrainComm() 
{
    stop();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool BrainComm::start() 
{
    if(running)
    {
        std::cerr << "Brain read and write threads are already running" << std::endl; 
        return true;
    }
    
    try{  

        running = true;  
        read_thread = make_unique<thread>(&BrainComm::readLoop, this);
        write_thread = make_unique<thread>(&BrainComm::writeLoop, this);
        return true;

    } catch (const std::exception& e) 
    {
        std::cerr << "Failed to start Brain read and write threads: " << e.what() << std::endl;
        running = false;
        return false;  
    }
    
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void BrainComm::stop() 
{
    
    
    // Cancel any pending operations on the serial port
    if (serial_port && serial_port->is_open()) {
        serial_port->cancel();
    }
    running = false;
    // Wait for threads to finish
    if (read_thread && read_thread->joinable()) {
        read_thread->join();
    }
    read_thread.reset();
    
    if (write_thread && write_thread->joinable()) {
        write_thread->join();
    }
    write_thread.reset();
    
    if (serial_port && serial_port->is_open()) {
        serial_port->close();
    }
    connected = false;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool BrainComm::restart() 
{
    stop();
    return start();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool BrainComm::reconnect() 
{
    if (serial_port && serial_port->is_open())
    {
        serial_port->close();
    }
    std::cerr << "Attempting to reconnect VEX Brain..." << std::endl;
    return findPort();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool BrainComm::initializePort() 
{
    if(port == "")
    {
        std::cerr << "Can not initialize Brain port, no port detected" << std::endl;
        return false;
    }
   
    
    try {
        serial_port = make_unique<boost::asio::serial_port>(io_service);
        serial_port->open(port);
        serial_port->set_option(boost::asio::serial_port_base::baud_rate(115200));
        serial_port->set_option(boost::asio::serial_port_base::character_size(8));
        serial_port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        
        initialized = true;
        stats.recordConnectionStatus(true);
        cerr << "Brain communication initialized on port " << port << endl;
        return true;

    } catch (const boost::system::system_error& e) 
    {
        stats.logError(CommError::ConnectionLost, "Failed to initialize port: " + string(e.what()));
        cerr << "Failed to initialize port: " << endl ;
        initialized = false;
        return false;
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool BrainComm::updateRequests(uint16_t flags) 
{
    lock_guard<mutex> lock(state_mutex);
    pending_requests.push(flags);
    cerr << "Request queued with flags: 0x" << hex << flags << dec << endl;
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool BrainComm::findPort() 
{
    // Try to find Brain ports
    std::vector<std::string> brain_ports = PortDetector::findBrainPorts();

    if (port == "")
    {
        if(brain_ports.size() < 1)
        {
            std::cerr << "No VEX Brain detected via USB could not set port" << std::endl;
            connected = false;
            
        }

        else if(brain_ports.size() > 1)
        {
             std::cerr << "Too many VEX Brain Ports detected" << std::endl;
             connected = false;
        }
        else
        {
            std::cerr << "VEX Brain detected" << std::endl;
            port = brain_ports[0];
            connected = true;
            initializePort();
        }
    }
    else
    {
        if(brain_ports.size() < 1)
        {
            std::cerr << "No VEX Brain detected via USB could not find previously set port" << std::endl;
            connected = false;
        }
        else
        {
            
            for(int i = 0; i < brain_ports.size(); i++)
            {
                if(brain_ports[i] == port)
                {
                    std::cerr << "Reconnected to previously set port" << std::endl;
                    connected = true;
                    initializePort();
                    break;
                }
                else 
                    connected = false;

            }
            if(!connected)
            {
                std::cerr << "VEX Brain detected but the port has changed, (" <<  port 
                          << " -> " << brain_ports[0] << ")"  << std::endl;
                port = brain_ports[0];
                connected = true;
                initializePort();

            }
        }
    }

    return connected;

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void BrainComm::readLoop() 
{
    cerr << "Brain read loop started" << endl;
    vector<uint8_t> buffer(CommConstants::MAX_BUFFER_SIZE);
    size_t buffer_index = 0;
    
    while (running) 
    {

        if (!connected || !initialized)
        {
            reconnect();
            this_thread::sleep_for(CommConstants::RECONNECT_DELAY);
            continue;
        }
            
        // Create a small buffer for reading one byte at a time
        uint8_t read_byte;
        boost::system::error_code ec;
            
        // Read a single byte
        size_t bytes_read = boost::asio::read(*serial_port, 
                            boost::asio::buffer(&read_byte, 1),ec);
    
        if (ec) 
        {
            if (ec != boost::asio::error::operation_aborted ) 
            {
                cerr << "Read error: " << ec.message() << endl;
                stats.incrementReceiveStats(false);
                lock_guard<mutex> lock(state_mutex);
                connected = false;
            }

            this_thread::sleep_for(chrono::milliseconds(10));
            continue;
        }
            
        if (bytes_read > 0) 
        {
    
            if (buffer_index < CommConstants::MAX_BUFFER_SIZE) 
            {
                buffer[buffer_index++] = read_byte;
            } 
            else 
            {
                stats.recordBufferOverrun();
                buffer_index = 0;
                continue;
            }
               
                
            // Now process the buffer looking for complete messages
            bool found_start = false;
            size_t start_pos = 0;
                
            // Search for start marker
            if (buffer_index >= 2) 
            {  // Prevent underflow
                for (size_t i = 0; i <= buffer_index - 2; i++) 
                {
                    if (buffer[i] == CommConstants::START_MARKER_1 && 
                        buffer[i+1] == CommConstants::START_MARKER_2) 
                    {
                        found_start = true;
                        start_pos = i;
                        break;
                    }
                }
            }
                
            // If start marker is not at the beginning, shift buffer
            if (found_start && start_pos > 0) 
            {
                memmove(buffer.data(), buffer.data() + start_pos, buffer_index - start_pos);
                buffer_index -= start_pos;
                start_pos = 0;
            }
               
                
            // If we have enough data for a header
            if (found_start && buffer_index >= sizeof(RequestHeader)) 
            {
                RequestHeader* header = reinterpret_cast<RequestHeader*>(buffer.data());
                size_t total_length = sizeof(RequestHeader) + header->length + sizeof(EndMarker);
                
                if (buffer_index >= total_length) 
                {
                    // Verify end marker
                    EndMarker* end = reinterpret_cast<EndMarker*>(buffer.data() + sizeof(RequestHeader) 
                                                                                + header->length);
                        
                    if (end->marker[0] == CommConstants::END_MARKER_1 && 
                        end->marker[1] == CommConstants::END_MARKER_2) 
                    {
                            
                        // Process the message
                        handleMessage(buffer.data(), total_length);
                        // Move any remaining data to the start of the buffer
                        if (buffer_index > total_length) 
                        {
                            memmove(buffer.data(), buffer.data() + total_length, 
                                    buffer_index - total_length);
                            buffer_index -= total_length;
                        } 
                        else 
                        {
                            buffer_index = 0;
                        }
                    } 

                    else 
                    {
                        cerr << "Invalid end marker in message" << endl;
                        // Shift past the start marker to look for next valid message
                        if (buffer_index > 2) 
                        {
                            memmove(buffer.data(), buffer.data() + 2, buffer_index - 2);
                            buffer_index -= 2;
                        } 
                        else 
                        {
                            buffer_index = 0;
                        }
                    }
                }
            }
        }
    }
    
    cerr << "Read thread stopped" << endl;

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void BrainComm::writeLoop() 
{
    cerr << "Brain write loop started" << endl;

    uint16_t current_request;
    uint16_t current_response;
    uint16_t current_acknocknowledgment;
    bool send_request;
    bool send_data;
    bool send_acknowledgment;
    auto current_time = chrono::duration_cast<std::chrono::milliseconds>(
                        chrono::steady_clock::now().time_since_epoch()).count();
    
    while (running) 
    {
        if (!connected || !initialized)
        {
            this_thread::sleep_for(CommConstants::RECONNECT_DELAY);
            continue;
        }


        current_request = 0;
        current_response = 0;
        current_acknocknowledgment = 0;
        send_request = false;
        send_data = false;
        send_acknowledgment = false;

        current_time = chrono::duration_cast<std::chrono::milliseconds>(
                        chrono::steady_clock::now().time_since_epoch()).count();
        
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            if((current_time - last_request_time  >= CommConstants::REQUEST_RETRY_PERIOD.count()))
            {
                if(!request_in_progress)
                {
                    if (!pending_requests.empty()) 
                    {
                        current_request = pending_requests.front();
                        send_request = true;
                        request_in_progress = true;
                    }
                }

                else 
                {
                    current_request = pending_requests.front();
                    send_request = true;
                }
            }
            
            if(response_flags != static_cast<uint16_t>(Brain::RequestFlag::NoData))
            {
                current_response = response_flags;
                send_data = true;
            }
            else 
            {
                current_response = response_flags;
                send_data = false;
            }

            if(!pending_acknowledgments.empty())
            {
                current_acknocknowledgment = pending_acknowledgments.front();
                send_acknowledgment = true;
            }
        }
            
        if(send_request)
        {
            sendRequests(current_request);
        }

        if(send_acknowledgment)
        {
            sendAcknowledgment(current_acknocknowledgment, 0x00);
        }

        current_time = chrono::duration_cast<std::chrono::milliseconds>(
                       chrono::steady_clock::now().time_since_epoch()).count();
        
        if(send_data && (current_time - last_request_time >= CommConstants::RESPONSE_UPDATE_PERIOD.count()))
        {
            sendResponse(current_response);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    std::cerr << "Write thread stopped" << std::endl;

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool BrainComm::handleMessage(const uint8_t* buffer, size_t length) {
    // Ensure we have at least a header
    if (length < sizeof(RequestHeader)) 
    {
        std::cerr << "Message too short for header" << std::endl;
        return false;
    }
    
    const RequestHeader* header = reinterpret_cast<const RequestHeader*>(buffer);
    
    // Verify start marker
    if (header->start_marker[0] != CommConstants::START_MARKER_1 || 
        header->start_marker[1] != CommConstants::START_MARKER_2) 
    {
        std::cerr << "Invalid start marker in message" << std::endl;
        return false;
    }
    
    // Calculate total expected length
    size_t expected_length = sizeof(RequestHeader) + header->length + sizeof(EndMarker);
    if (length < expected_length) {
        std::cerr << "Message shorter than expected length" << std::endl;
        return false;
    }
    
    // Verify end marker
    const EndMarker* end = reinterpret_cast<const EndMarker*>(
        buffer + sizeof(RequestHeader) + header->length);
        
    if (end->marker[0] != CommConstants::END_MARKER_1 || 
        end->marker[1] != CommConstants::END_MARKER_2) 
    {
        std::cerr << "Invalid end marker in message" << std::endl;
        return false;
    }
    
    // Process based on message type
    MessageType msg_type = static_cast<MessageType>(header->message_type);
    uint16_t flags = header->flags;
    
    switch (msg_type) {
        case MessageType::Request:

            std::cerr << "Received request message with flags: 0x" 
                     << std::hex << flags << std::dec << std::endl;
            
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                response_flags = flags;
                pending_acknowledgments.push(flags);
                last_received_time = chrono::duration_cast<std::chrono::milliseconds>(
                                     chrono::steady_clock::now().time_since_epoch()).count();
            }
            stats.incrementReceiveStats(true);
            break;
            
        case MessageType::Acknowledgment:

            std::cerr << "Received acknowledgment message for flags: 0x" 
                      << std::hex << flags << dec << endl;
                
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                if (request_in_progress && !pending_requests.empty())
                {
                    if(pending_requests.front() == flags)
                    {
                        request_flags = flags;              // Store what we requested
                        pending_requests.pop();             // Remove current request from queue
                        request_in_progress = false;        // Stop any current request
                        request_retry_count = 0;
                        last_received_time = chrono::duration_cast<std::chrono::milliseconds>(
                                             chrono::steady_clock::now().time_since_epoch()).count();
                    }
                }
            }
            stats.incrementReceiveStats(true);
            break;
            
        case MessageType::Response:
            
            if (!processReceivedData(flags, buffer + sizeof(RequestHeader), header->length)) 
            {
                stats.incrementReceiveStats(false);
                return false;
            }
            else
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                last_received_time = chrono::duration_cast<std::chrono::milliseconds>(
                                     chrono::steady_clock::now().time_since_epoch()).count();
            }
            
            stats.incrementReceiveStats(true);
            break;
            
            
        default:
            std::cerr << "Unknown message type: " 
                     << static_cast<int>(header->message_type) << std::endl;
            stats.incrementReceiveStats(false);
            return false;
    }
    
    return true;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void BrainComm::sendRequests(uint16_t flags) 
{

    RequestHeader header;
    header.start_marker[0] = CommConstants::START_MARKER_1;
    header.start_marker[1] = CommConstants::START_MARKER_2;
    header.message_type = static_cast<uint8_t>(MessageType::Request);
    header.flags = flags;
    header.length = 0;
    
    EndMarker end_marker;
    end_marker.marker[0] = CommConstants::END_MARKER_1;
    end_marker.marker[1] = CommConstants::END_MARKER_2;
    

    boost::system::error_code ec;
    vector<boost::asio::const_buffer> buffers;

    buffers.push_back(boost::asio::buffer(&header, sizeof(header)));
    buffers.push_back(boost::asio::buffer(&end_marker, sizeof(end_marker)));
    boost::asio::write(*serial_port, buffers, ec);

    if (!ec) 
    {
        
        {
            lock_guard<mutex> lock(state_mutex);
            cerr << "Request sent with flags: 0x" << hex << flags << dec 
                 << ", attempt: " << static_cast<int>(request_retry_count) << ")" << endl; 
            last_request_time = chrono::duration_cast<chrono::milliseconds>(
                                chrono::steady_clock::now().time_since_epoch()).count();
            request_retry_count++;
            if(request_retry_count  >= CommConstants::MAX_REQUEST_RETRIES)
            {
                if (!pending_requests.empty()) 
                {
                    cerr << "Request timed out after " << CommConstants::MAX_REQUEST_RETRIES 
                        << " retries (flags: 0x" << hex << pending_requests.front() 
                        << dec << ")" << endl;
                    pending_requests.pop();
                    request_in_progress = false;
                    request_retry_count = 0;
                }
            }
        }
        stats.incrementTransmitStats(true);
    }
    else
    {
        cerr << "Failed to send request with flags: 0x" << hex << flags << dec << endl;
        stats.incrementTransmitStats(false);
    }

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void BrainComm::sendResponse(uint16_t flags) {
    // Calculate total payload size based on active flags
    size_t payload_size = 0;
    
    if (flags & static_cast<uint16_t>(RequestFlag::MotorVoltages)) 
        payload_size += sizeof(MotorCommand);
        
    if (flags & static_cast<uint16_t>(RequestFlag::MacroControls)) 
        payload_size += sizeof(ControlFlags);

    if (flags & static_cast<uint16_t>(RequestFlag::BatteryLevel))
        payload_size += sizeof(uint32_t);
    
    // Prepare request header
    RequestHeader header;
    header.start_marker[0] = CommConstants::START_MARKER_1;
    header.start_marker[1] = CommConstants::START_MARKER_2;
    header.message_type = static_cast<uint8_t>(MessageType::Response);
    header.flags = flags;
    header.length = payload_size;
    
    // Set up end marker
    EndMarker end_marker;
    end_marker.marker[0] = CommConstants::END_MARKER_1;
    end_marker.marker[1] = CommConstants::END_MARKER_2;
    
    // Prepare payload buffer
    vector<uint8_t> payload(payload_size);
    size_t offset = 0;
    
    {
        lock_guard<mutex> lock(data_mutex);
        
        // Motor voltages
        if (flags & static_cast<uint16_t>(RequestFlag::MotorVoltages)) {
            // Update timestamp before sending
            current_motor_command.timestamp = chrono::duration_cast<chrono::milliseconds>(
                chrono::steady_clock::now().time_since_epoch()).count();
                
            memcpy(payload.data() + offset, &current_motor_command, sizeof(MotorCommand));
            offset += sizeof(MotorCommand);
        }
        // Macro controls
        if (flags & static_cast<uint16_t>(RequestFlag::MacroControls)) 
        {
            memcpy(payload.data() + offset, &current_control_flags, sizeof(ControlFlags));
            offset += sizeof(ControlFlags);
        }
        // Jetson Battery Level
        if (flags & static_cast<uint16_t>(RequestFlag::BatteryLevel)) 
        {
            memcpy(payload.data() + offset, &current_battery_lvl, sizeof(uint32_t));
            offset += sizeof(uint32_t);
        }

    }
    
    // Send header, payload, and end marker using async_write
    vector<boost::asio::const_buffer> buffers;
    buffers.push_back(boost::asio::buffer(&header, sizeof(header)));
    
    if (payload_size > 0) {
        buffers.push_back(boost::asio::buffer(payload));
    }
    
    buffers.push_back(boost::asio::buffer(&end_marker, sizeof(end_marker)));

    boost::system::error_code ec;
    boost::asio::write(*serial_port, buffers, ec);

    if (!ec) 
    {
        // cerr << "Response sent with flags: 0x" << hex << flags 
        //      << dec << ", payload size: " << payload_size << endl;
        last_send_time = chrono::duration_cast<chrono::milliseconds>(
                         chrono::steady_clock::now().time_since_epoch()).count();
        stats.incrementTransmitStats(true);
    }
    else
    {
        cerr << "Failed to send response with flags: 0x" << hex << flags 
             << dec << ", payload size: " << payload_size << endl;
        stats.incrementTransmitStats(false);
    }
    
             
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void BrainComm::sendAcknowledgment(uint16_t flags, uint8_t status) 
{
    // Create a header with flags, message type, and length=1
    RequestHeader header;
    header.start_marker[0] = CommConstants::START_MARKER_1;
    header.start_marker[1] = CommConstants::START_MARKER_2;
    header.message_type = static_cast<uint8_t>(MessageType::Acknowledgment);
    header.flags = flags;
    header.length = 1;  // 1 byte payload for status
    
    // Set up end marker
    EndMarker end_marker;
    end_marker.marker[0] = CommConstants::END_MARKER_1;
    end_marker.marker[1] = CommConstants::END_MARKER_2;
    
    // Use scatter-gather approach for efficiency
    vector<boost::asio::const_buffer> buffers;
    buffers.push_back(boost::asio::buffer(&header, sizeof(header)));
    buffers.push_back(boost::asio::buffer(&status, sizeof(status)));
    buffers.push_back(boost::asio::buffer(&end_marker, sizeof(end_marker)));
    
    boost::system::error_code ec;
    boost::asio::write(*serial_port, buffers, ec);
    
    if (!ec) 
    {
        cerr << "Acknowledgment sent with flags: 0x" << hex << flags << dec << endl;
        {
            lock_guard<mutex> lock(data_mutex);
            pending_acknowledgments.pop();
        }
        stats.incrementTransmitStats(true);
    }
    else
    {
        cerr << "Failed to send acknowledgment with flags: 0x" << hex << flags << dec << endl;
        stats.incrementTransmitStats(false);
    }
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool BrainComm::processReceivedData(uint16_t flags, const uint8_t* data, uint16_t length) {
    // Calculate expected payload size based on flags
    size_t expected_size = 0;
    if (flags & static_cast<uint16_t>(RequestFlag::LeftGPSData))
        expected_size += sizeof(Position2D) * 2;  // position and offset
    if (flags & static_cast<uint16_t>(RequestFlag::RightGPSData))
        expected_size += sizeof(Position2D) * 2;  // position and offset
    if (flags & static_cast<uint16_t>(RequestFlag::SisterData))
        expected_size += sizeof(Position2D);
    if (flags & static_cast<uint16_t>(RequestFlag::BatteryLevel))
        expected_size += sizeof(uint32_t);
    
    // Validate total payload size
    if (length < expected_size) {
        stats.logError(CommError::InvalidPacket, "Received data size mismatch");
        cerr << "Data packet size mismatch. Expected: " << expected_size 
             << ", got: " << length << endl;
        return false;
    }
    
    // Lock the data mutex to safely update stored values
    lock_guard<mutex> lock(data_mutex);
    
    size_t offset = 0;
    
    // Process left GPS data
    if (flags & static_cast<uint16_t>(RequestFlag::LeftGPSData)) {
        if (offset + sizeof(Position2D) <= length) {
            memcpy(&left_gps_position, data + offset, sizeof(Position2D));
            offset += sizeof(Position2D);
            
            // Get the offset data
            if (offset + sizeof(Position2D) <= length) {
                memcpy(&left_gps_offset, data + offset, sizeof(Position2D));
                offset += sizeof(Position2D);
                
                // cerr << "Received left GPS position: (" 
                //      << left_gps_position.x << ", " 
                //      << left_gps_position.y << ", " 
                //      << left_gps_position.heading << ")" << endl;
            }
        }
    }
    
    // Process right GPS data
    if (flags & static_cast<uint16_t>(RequestFlag::RightGPSData)) {
        if (offset + sizeof(Position2D) <= length) {
            memcpy(&right_gps_position, data + offset, sizeof(Position2D));
            offset += sizeof(Position2D);
            
            // Get the offset data
            if (offset + sizeof(Position2D) <= length) {
                memcpy(&right_gps_offset, data + offset, sizeof(Position2D));
                offset += sizeof(Position2D);
                
                // cerr << "Received right GPS position: (" 
                //      << right_gps_position.x << ", " 
                //      << right_gps_position.y << ", " 
                //      << right_gps_position.heading << ")" << endl;
            }
        }
    }
    
    // Process sister robot data
    if (flags & static_cast<uint16_t>(RequestFlag::SisterData)) {
        if (offset + sizeof(Position2D) <= length) {
            memcpy(&sister_position, data + offset, sizeof(Position2D));
            offset += sizeof(Position2D);
            
            // cerr << "Received sister position: (" 
            //      << sister_position.x << ", " 
            //      << sister_position.y << ", " 
            //      << sister_position.heading << ")" << endl;
        }
    }
    
    // Process battery level
    if (flags & static_cast<uint16_t>(RequestFlag::BatteryLevel)) 
    {
        if (offset + sizeof(uint32_t) <= length) {
            memcpy(&BrainBatteryLvl, data + offset, sizeof(uint32_t));
            offset += sizeof(uint32_t);
            
            // cerr << "Received Brain battery level: " 
            //      << BrainBatteryLvl << "%" << endl;
        }
    }

    stats.incrementReceiveStats(true);
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void BrainComm::setMotorVoltages(float left, float right) 
{
    lock_guard<mutex> lock(data_mutex);
    current_motor_command.left_voltage = left;
    current_motor_command.right_voltage = right;
    current_motor_command.timestamp = 
        chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void BrainComm::setMacroBits(uint32_t macro_bits) 
{
    lock_guard<mutex> lock(data_mutex);
    current_control_flags.macro_bits = macro_bits;
}

void BrainComm::setJetsonBattery(uint32_t level)
{
    lock_guard<mutex> lock(data_mutex);
    current_battery_lvl = level;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


Position2D BrainComm::getLeftGPSData() const 
{
    lock_guard<mutex> lock(data_mutex);
    return left_gps_position;
}

Position2D BrainComm::getRightGPSData() const 
{
    lock_guard<mutex> lock(data_mutex);
    return right_gps_position;
}

Position2D BrainComm::getSisterPosition() const 
{
    lock_guard<mutex> lock(data_mutex);
    return sister_position;
}

Position2D BrainComm::getLeftGPSOffset() const 
{
    lock_guard<mutex> lock(data_mutex);
    return left_gps_offset;
}

Position2D BrainComm::getRightGPSOffset() const 
{
    lock_guard<mutex> lock(data_mutex);
    return right_gps_offset;
}

uint32_t BrainComm::getBrainBattery() const 
{
    lock_guard<mutex> lock(data_mutex);
    return BrainBatteryLvl;
}

} // namespace Brain