// #include "vex.h"
#include "robot-config.h"
#include "JetsonComm.h"

namespace Jetson {

JetsonComm::JetsonComm() : 
    running(true),
    request_in_progress(false),
    request_retry_count(0),
    request_flags(static_cast<uint16_t>(RequestFlag::NoData)),
    response_flags(static_cast<uint16_t>(RequestFlag::NoData)),
    last_send_time(0),
    last_received_time(0),
    last_request_time(0),
    JetsonBatteryLvl(0)
    
    
{
    // Start communication threads
    serial_port = fopen("/dev/serial1", "w+");
    read_thread = new vex::thread(readThread, static_cast<void*>(this));
    write_thread = new vex::thread(writeThread, static_cast<void*>(this));
    fprintf(fp, "JetsonComm initialized\n");
}

JetsonComm::~JetsonComm() 
{
    if (serial_port != NULL)
    {
        fclose(serial_port);
    }
    running = false;
    delete read_thread;
    delete write_thread;
}


void JetsonComm::updateRequests(uint16_t flags) 
{
    state_mutex.lock();
    pending_requests.push(flags);
    state_mutex.unlock();
    fprintf(fp, "Request queued with flags: 0x%04X\n\r", flags);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int JetsonComm::writeThread(void* arg) 
{
    JetsonComm* instance = static_cast<JetsonComm*>(arg);
    uint16_t current_request;
    uint16_t current_response;
    uint16_t current_acknocknowledgment;
    bool send_request;
    bool send_data;
    bool send_acknowledgment;
    uint32_t current_time;
    
    while (instance->running) 
    {
        current_request = 0;
        current_response = 0;
        current_acknocknowledgment = 0;
        send_request = false;
        send_data = false;
        send_acknowledgment = false;
        current_time = instance->timer.system();

        {
            instance->state_mutex.lock();
            if((current_time - instance->last_request_time >= CommConstants::REQUEST_RETRY_PERIOD))
            {
                if (!instance->request_in_progress)
                {
                    if(!instance->pending_requests.empty()) 
                    {
                        current_request = instance->pending_requests.front();
                        send_request = true;
                        instance->request_in_progress = true;
                    }
                }
                else
                {
                    current_request = instance->pending_requests.front();
                    send_request = true;
                }
            }

            if(instance->response_flags != static_cast<uint16_t>(Jetson::RequestFlag::NoData))
            {
                current_response = instance->response_flags;
                send_data = true;
            }
            else 
            {
                current_response = instance->response_flags;
                send_data = false;

            }

            if(!instance->pending_acknowledgments.empty())
            {
                current_acknocknowledgment = instance->pending_acknowledgments.front();
                send_acknowledgment = true;
            }
            instance->state_mutex.unlock();
        }

        if (send_request)
        {
            instance->sendRequests(current_request);
        }
        
        if(send_acknowledgment)
        {
            instance->sendAcknowledgment(current_acknocknowledgment,0x00);
        }

        current_time = instance->timer.system();

        if(send_data && (current_time - instance->last_send_time >= CommConstants::RESPONSE_UPDATE_PERIOD))
        {
            instance->sendResponse(current_response);
        }

        vex::this_thread::sleep_for(5);
    }

    fprintf(fp, "Write thread stopped\n\r");
    return 0;
       
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int JetsonComm::readThread(void* arg) 
{
    JetsonComm* instance = static_cast<JetsonComm*>(arg);
    uint8_t buffer[CommConstants::MAX_BUFFER_SIZE];
    size_t buffer_index = 0;

    while (instance->running) 
    {
        int c = getchar();
        if (c < 0) 
        {
            vex::this_thread::sleep_for(1);
            continue;
        }

        // Add byte to buffer
        buffer[buffer_index++] = static_cast<uint8_t>(c);
        
        bool found_start = false;
        size_t start_pos = 0;

        if(buffer_index >=2) // Prevent underflow
        {   
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
        if(found_start && start_pos > 0)
        {
            memmove(buffer, buffer + start_pos, buffer_index - start_pos);
            buffer_index -= start_pos;
            start_pos = 0;
        }
                
        // If no start marker found, continue reading (but prevent buffer overflow)
        if (!found_start && buffer_index >= CommConstants::MAX_BUFFER_SIZE) 
        {
            //fprintf(fp, "No start marker found, buffer full, resetting\n\r");
            buffer_index = 0;
            continue;
        }
        
        // Process complete header if we have enough data
        if (found_start && buffer_index >= sizeof(RequestHeader)) 
        {
            RequestHeader* header = reinterpret_cast<RequestHeader*>(buffer);
            size_t total_length = sizeof(RequestHeader) + header->length + sizeof(EndMarker);       // Calculate total message length including header, payload, and end marker
            
            // Ensure we have the full packet
            if (buffer_index >= total_length) 
            {
                // Check end marker
                EndMarker* end = reinterpret_cast<EndMarker*>(buffer + sizeof(RequestHeader) 
                                                                     + header->length);
                    
                if (end->marker[0] == CommConstants::END_MARKER_1 && 
                    end->marker[1] == CommConstants::END_MARKER_2) 
                {
                    instance->handleMessage(buffer,total_length);

                    // Move any remaining data to start of buffer
                    if (buffer_index > total_length) 
                    {
                        memmove(buffer, buffer + total_length, buffer_index - total_length);
                        buffer_index -= total_length;
                        fprintf(fp, "Moved %zu remaining bytes to buffer start\n\r", buffer_index);
                    }
                    else
                    {
                        buffer_index = 0;
                    }
                }

                else
                {
                    fprintf(fp, "Invalid end marker in message\n\r");
                    if(buffer_index > 2)
                    {
                        memmove(buffer, buffer + 2, buffer_index - 2);
                        buffer_index -=2;
                    }
                    else
                    {
                        buffer_index = 0;
                    }
                }
            }
        }
        
        // Buffer overflow protection
        if (buffer_index >= CommConstants::MAX_BUFFER_SIZE) 
        {
            instance->stats.recordBufferOverrun();
            instance->stats.logError(CommError::BufferOverrun, "Receive buffer overflow");
            fprintf(fp, "Buffer overflow in read thread\n\r");
            buffer_index = 0;
        }
    }

    fprintf(fp, "Read thread stopped\n\r");
    return 0;

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool JetsonComm::handleMessage(const uint8_t* buffer, size_t length)
{
    if(length < sizeof(RequestHeader))
    {
        fprintf(fp, "Message too short for header\n\r");
        stats.recordReception(false, length);
        return false;
    }

    const RequestHeader* header = reinterpret_cast<const RequestHeader*>(buffer);

    // Verify start marker
    if (header->start_marker[0] != CommConstants::START_MARKER_1 || 
    header->start_marker[1] != CommConstants::START_MARKER_2) 
    {
        fprintf(fp, "Invalid start marker in message\n\r");
        stats.recordReception(false, length);
        return false;
    }

    // Calculate total expected length
    size_t expected_length = sizeof(RequestHeader) + header->length + sizeof(EndMarker);
    if (length < expected_length) 
    {
        fprintf(fp, "Message shorter than expected length\n\r");
        stats.recordReception(false, length);
        return false;
    }

    // Verify end marker
    const EndMarker* end = reinterpret_cast<const EndMarker*>(buffer + sizeof(RequestHeader) 
                                                                     + header->length);
    if (end->marker[0] != CommConstants::END_MARKER_1 || 
        end->marker[1] != CommConstants::END_MARKER_2) 
    {
        fprintf(fp, "Invalid end marker in message\n\r");
        return false;
    }

    // Process based on message type
    MessageType msg_type = static_cast<MessageType>(header->message_type);
    uint16_t flags = header->flags;
    
    switch (msg_type) 
    {
      
        case MessageType::Request:
            fprintf(fp, "Received request message with flags: 0x%04X\n\r", flags);
            
        {
            state_mutex.lock();
            response_flags = flags;
            pending_acknowledgments.push(flags);
            last_received_time = timer.system();
            state_mutex.unlock();
        }
        stats.recordReception(true, length);
        break;
            
        case MessageType::Acknowledgment:
             
            fprintf(fp, "Received acknowledgment for flags: 0x%04X\n\r", flags);
            {
                state_mutex.lock();
                if (request_in_progress && !pending_requests.empty())
                {
                    if(pending_requests.front() == flags)
                    {
                        request_flags = flags;
                        pending_requests.pop();
                        request_in_progress = false;
                        request_retry_count = 0;
                        last_received_time = timer.system();
                        
                    }
                }
                state_mutex.unlock();
            }
            stats.recordReception(true, length);
            break;
            
        case MessageType::Response:
        
            if(!processReceivedData(flags, buffer + sizeof(RequestHeader), header->length))
            {
                stats.recordReception(false, length);
                return false;
            }
            else
            {
                state_mutex.lock();
                last_received_time = timer.system();
                state_mutex.unlock();

            }
            stats.recordReception(true, length);
            break;
            
        default:
            fprintf(fp, "Unknown message type: %d\n\r", static_cast<int>(header->message_type));
            stats.recordReception(true, length);
            stats.logError(CommError::InvalidPacket, "Received unknown message type");
            return false;
         
    }

    return true;
}





//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



bool JetsonComm::sendRequests(uint16_t flags) 
{
    // Create a single buffer for the complete message
    uint8_t buffer[CommConstants::MAX_BUFFER_SIZE];
    size_t buffer_index = 0;
    
    // Build header in buffer
    RequestHeader* header = reinterpret_cast<RequestHeader*>(buffer);
    header->start_marker[0] = CommConstants::START_MARKER_1;
    header->start_marker[1] = CommConstants::START_MARKER_2;
    header->message_type = static_cast<uint8_t>(MessageType::Request);
    header->flags = flags;
    header->length = 0;  // No payload for requests
    buffer_index += sizeof(RequestHeader);
    
    // Add end marker to buffer
    EndMarker* end_marker = reinterpret_cast<EndMarker*>(buffer + buffer_index);
    end_marker->marker[0] = CommConstants::END_MARKER_1;
    end_marker->marker[1] = CommConstants::END_MARKER_2;
    buffer_index += sizeof(EndMarker);
    
    // Send the complete message at once
    bool success = false;
    if (serial_port != NULL) 
    {
        if (fwrite(buffer, 1, buffer_index, serial_port) == buffer_index) 
        {
            fflush(serial_port);
            success = true;
        }
    }

    if (success) 
    {
       
        state_mutex.lock();
        fprintf(fp, "Request sent with flags: 0x%04X, attempt: %i)\n\r", flags, static_cast<int>(request_retry_count));
        last_request_time = timer.system();
        request_retry_count++;
        if(request_retry_count >= CommConstants::MAX_REQUEST_RETRIES)
        {
            if(!pending_requests.empty())
            {
                fprintf(fp, "Request timed out after %zu retries (flags: 0x%04X)\n\r",CommConstants::MAX_REQUEST_RETRIES,pending_requests.front());
                pending_requests.pop();
                request_in_progress =false;
                request_retry_count = 0;
                stats.recordRequestTimeout();
                stats.logError(CommError::RequestTimeout, "Request timed out");

            }
        }
        state_mutex.unlock();
        stats.recordTransmission(success, buffer_index); 
        
    } 
    else 
    {
        stats.recordTransmission(success, buffer_index); 
        stats.logError(CommError::TransmissionFailed, "Failed to send request");
        fprintf(fp, "Failed to send request with flags: 0x%04X\n\r", flags);
    }
    return success;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool JetsonComm::sendAcknowledgment(uint16_t flags, uint8_t status) 
{
    // Create a single buffer for the complete message
    uint8_t buffer[CommConstants::MAX_BUFFER_SIZE];
    size_t buffer_index = 0;
    
    // Build header in buffer
    RequestHeader* header = reinterpret_cast<RequestHeader*>(buffer);
    header->start_marker[0] = CommConstants::START_MARKER_1;
    header->start_marker[1] = CommConstants::START_MARKER_2;
    header->message_type = static_cast<uint8_t>(MessageType::Acknowledgment);
    header->flags = flags;
    header->length = 1;  // 1 byte payload for status
    buffer_index += sizeof(RequestHeader);
    
    // Add status byte to buffer
    buffer[buffer_index++] = status;
    
    // Add end marker to buffer
    EndMarker* end_marker = reinterpret_cast<EndMarker*>(buffer + buffer_index);
    end_marker->marker[0] = CommConstants::END_MARKER_1;
    end_marker->marker[1] = CommConstants::END_MARKER_2;
    buffer_index += sizeof(EndMarker);
    
    // Send the complete message at once
    bool success = false;
    if (serial_port != NULL) 
    {
        if (fwrite(buffer, 1, buffer_index, serial_port) == buffer_index) 
        {
            fflush(serial_port);
            success = true;
        }
    }

    if (success) 
    {
        stats.recordTransmission(success, buffer_index); 
        state_mutex.lock();
        last_send_time = timer.system();
        
        // Successfully sent, remove from pending acknowledgments
        if (!pending_acknowledgments.empty()) {
            pending_acknowledgments.pop();
        }
        state_mutex.unlock();
        
        fprintf(fp, "Acknowledgment sent for flags: 0x%04X\n\r", flags);
    } 
    else
    {
        stats.recordTransmission(success, buffer_index); 
        stats.logError(CommError::TransmissionFailed, "Failed to send acknowledgment");
        fprintf(fp, "Failed to send acknowledgment for flags: 0x%04X\n\r", flags);
    }
    return success;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool JetsonComm::sendResponse(uint16_t flags) 
{
    // Create a single buffer for the complete message
    uint8_t buffer[CommConstants::MAX_BUFFER_SIZE];
    size_t buffer_index = 0;
    
    // Calculate payload size based on flags
    size_t payload_size = 0;
    if (flags & static_cast<uint16_t>(RequestFlag::LeftGPSData)) 
        payload_size += sizeof(Position2D) * 2;  // Position2D and offset
    if (flags & static_cast<uint16_t>(RequestFlag::RightGPSData)) 
        payload_size += sizeof(Position2D) * 2;  // Position2D and offset
    if (flags & static_cast<uint16_t>(RequestFlag::SisterData)) 
        payload_size += sizeof(Position2D);
    if (flags & static_cast<uint16_t>(RequestFlag::BatteryLevel)) 
        payload_size += sizeof(uint32_t);
    
    // Build header in buffer
    RequestHeader* header = reinterpret_cast<RequestHeader*>(buffer);
    header->start_marker[0] = CommConstants::START_MARKER_1;
    header->start_marker[1] = CommConstants::START_MARKER_2;
    header->message_type = static_cast<uint8_t>(MessageType::Response);
    header->flags = flags;
    header->length = payload_size;
    buffer_index += sizeof(RequestHeader);
    
    // Add payload data to buffer
    if (flags & static_cast<uint16_t>(RequestFlag::LeftGPSData)) 
    {
        Position2D pos = {
            Left_GPS.xPosition(vex::distanceUnits::mm),
            Left_GPS.yPosition(vex::distanceUnits::mm),
            Left_GPS.heading(vex::rotationUnits::deg)
        };
        memcpy(buffer + buffer_index, &pos, sizeof(Position2D));
        buffer_index += sizeof(Position2D);
        memcpy(buffer + buffer_index, &LeftOffset, sizeof(Position2D));
        buffer_index += sizeof(Position2D);
    }
    if (flags & static_cast<uint16_t>(RequestFlag::RightGPSData)) 
    {
        Position2D pos = {
            Right_GPS.xPosition(vex::distanceUnits::mm),
            Right_GPS.yPosition(vex::distanceUnits::mm),
            Right_GPS.heading(vex::rotationUnits::deg)
        };
        memcpy(buffer + buffer_index, &pos, sizeof(Position2D));
        buffer_index += sizeof(Position2D);
        memcpy(buffer + buffer_index, &RightOffset, sizeof(Position2D));
        buffer_index += sizeof(Position2D);
    }
    if (flags & static_cast<uint16_t>(RequestFlag::SisterData)) 
    {
        Position2D sister_pos;
        link.get_remote_location(sister_pos.x, sister_pos.y, sister_pos.heading);
        memcpy(buffer + buffer_index, &sister_pos, sizeof(Position2D));
        buffer_index += sizeof(Position2D);
    }
    if (flags & static_cast<uint16_t>(RequestFlag::BatteryLevel)) 
    {
        uint32_t BatteryPer = Brain.Battery.capacity(vex::percentUnits::pct);
        memcpy(buffer + buffer_index, &BatteryPer, sizeof(uint32_t));
        buffer_index += sizeof(uint32_t);
    }
    
    // Add end marker to buffer
    EndMarker* end_marker = reinterpret_cast<EndMarker*>(buffer + buffer_index);
    end_marker->marker[0] = CommConstants::END_MARKER_1;
    end_marker->marker[1] = CommConstants::END_MARKER_2;
    buffer_index += sizeof(EndMarker);
    
    // Send the complete message at once
    bool success = false;
    if (serial_port != NULL) 
    {
        if (fwrite(buffer, 1, buffer_index, serial_port) == buffer_index) 
        {
            fflush(serial_port);
            success = true;
        }
    }

    if (success)
    {
        stats.recordTransmission(success, buffer_index);  
        state_mutex.lock();
        last_send_time = timer.system();
        state_mutex.unlock();
        // fprintf(fp, "Sent data packet with flags: 0x%04X\n\r", flags);
    }
    else
    {
        stats.recordTransmission(success, buffer_index);
        stats.logError(CommError::TransmissionFailed, "Failed to send data packet");
        fprintf(fp, "Failed to send data packet with flags: 0x%04X\n\r", flags);
    }
    return success;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool JetsonComm::processReceivedData(uint16_t flags, const uint8_t* data, uint16_t length) 
{
    // Calculate expected payload size based on flags
    size_t expected_size = 0;
    if (flags & static_cast<uint16_t>(RequestFlag::MotorVoltages)) 
        expected_size += sizeof(MotorCommand);
    if (flags & static_cast<uint16_t>(RequestFlag::MacroControls)) 
        expected_size += sizeof(ControlFlags);
    if (flags & static_cast<uint16_t>(RequestFlag::BatteryLevel)) 
        expected_size += sizeof(uint32_t);
    
    // Validate total payload size
    if (length < expected_size) 
    {
        fprintf(fp, "Data packet size mismatch. Expected: %zu, got: %d\n\r", expected_size, length);
        stats.logError(CommError::InvalidPacket, "Received data size mismatch");
        return false;
    }
    
    // Create local copies that we'll fill and then update under mutex protection
    MotorCommand motor_cmd;
    ControlFlags control_flags;
    uint32_t jetson_batt;
    
    // Read payload data into local variables
    size_t offset = 0;
    bool has_motor_data = false;
    bool has_control_data = false;
    bool has_battery_data = false;
    
    if (flags & static_cast<uint16_t>(RequestFlag::MotorVoltages)) 
    {
        if (offset + sizeof(MotorCommand) <= length) 
        {
            memcpy(&motor_cmd, data + offset, sizeof(MotorCommand));
            offset += sizeof(MotorCommand);
            has_motor_data = true;
        }
    }

    if (flags & static_cast<uint16_t>(RequestFlag::MacroControls)) 
    {
        if (offset + sizeof(ControlFlags) <= length) 
        {
            memcpy(&control_flags, data + offset, sizeof(ControlFlags));
            offset += sizeof(ControlFlags);
            has_control_data = true;
        }
    }

    if (flags & static_cast<uint16_t>(RequestFlag::BatteryLevel)) 
    {
        if (offset + sizeof(uint32_t) <= length) 
        {
            memcpy(&jetson_batt, data + offset, sizeof(uint32_t));
            offset += sizeof(uint32_t);
            has_battery_data = true;
        }
    }

    // Now update shared data under mutex protection
    data_mutex.lock();
    if (has_motor_data) {
        current_motor_cmd = motor_cmd;
    }
    if (has_control_data) {
        current_control_flags = control_flags;
    }
    if (has_battery_data) {
        JetsonBatteryLvl = jetson_batt;
    }
    data_mutex.unlock();
    
    // Update statistics and timestamps (under appropriate mutex)
    stats.recordReception(true, length);
    state_mutex.lock();
    last_received_time = timer.system();
    state_mutex.unlock();
    
    // Now we can safely do I/O without holding any locks
    if (has_motor_data) {
        fprintf(fp, "Motor voltages set - Left: %.2f V, Right: %.2f V, Timestamp: %lu\n\r", 
                motor_cmd.left_voltage, motor_cmd.right_voltage, motor_cmd.timestamp);
        LeftDriveSmart.spin(forward,motor_cmd.left_voltage,volt);
        RightDriveSmart.spin(forward,motor_cmd.right_voltage,volt);

    }
    if (has_control_data) {
        fprintf(fp, "Macro bits received: 0x%08lX\n\r", control_flags.macro_bits);
    }
    if (has_battery_data) {
        fprintf(fp, "Received Jetson Battery percentage: %.2lu\n\r", jetson_batt);
    }

    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



MotorCommand JetsonComm::getMotorCommand() const 
{
    data_mutex.lock();
    MotorCommand cmd = current_motor_cmd;
    data_mutex.unlock();
    return cmd;
}

ControlFlags JetsonComm::getControlFlags() const 
{
    data_mutex.lock();
    ControlFlags flags = current_control_flags;
    data_mutex.unlock();
    return flags;
}

uint32_t JetsonComm::getJetsonBattery() const
{
    data_mutex.lock();
    uint32_t battery_lvl = JetsonBatteryLvl;
    data_mutex.unlock();
    return battery_lvl;
}

} // namespace Jetson