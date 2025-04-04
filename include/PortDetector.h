#ifndef PORT_DETECTOR_H
#define PORT_DETECTOR_H

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <boost/asio.hpp>
#include <algorithm>
#include <map>

class PortDetector {
public:
    struct DeviceInfo {
        std::string port;
        std::string vendorId;
        std::string productId;
        std::string manufacturer;
        std::string product;
        std::string interface;
        std::string serialNumber;
        std::string devpath;
        int portNumber;
    };

    static std::vector<DeviceInfo> listAvailablePorts() {
        std::vector<DeviceInfo> devices;
        DIR* dir = opendir("/sys/class/tty");
        
        if (!dir) {
            std::cerr << "Failed to open /sys/class/tty" << std::endl;
            return devices;
        }

        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string name(entry->d_name);
            if (name.find("ttyACM") != std::string::npos) {
                DeviceInfo device;
                device.port = "/dev/" + name;
                device.portNumber = std::stoi(name.substr(6));
                
                std::string syspath = "/sys/class/tty/" + name;
                std::string devicepath = syspath + "/device";
                
                // Get the full USB device path to group interfaces
                char buf[256];
                ssize_t len = readlink(devicepath.c_str(), buf, sizeof(buf) - 1);
                if (len != -1) {
                    buf[len] = '\0';
                    device.devpath = std::string(buf);
                    // Extract just the USB address part (e.g., "1-2.3")
                    size_t pos = device.devpath.find("usb");
                    if (pos != std::string::npos) {
                        pos = device.devpath.find("/", pos);
                        if (pos != std::string::npos) {
                            device.devpath = device.devpath.substr(0, pos);
                        }
                    }
                }
                
                // Read interface information
                device.interface = readFile(devicepath + "/interface");
                
                // Walk up to find USB device information
                std::string parentPath = devicepath + "/..";
                while (true) {
                    std::string vendorPath = parentPath + "/idVendor";
                    std::string productPath = parentPath + "/idProduct";
                    
                    if (fileExists(vendorPath) && fileExists(productPath)) {
                        device.vendorId = readFile(vendorPath);
                        device.productId = readFile(productPath);
                        device.manufacturer = readFile(parentPath + "/manufacturer");
                        device.product = readFile(parentPath + "/product");
                        device.serialNumber = readFile(parentPath + "/serial");
                        break;
                    }
                    
                    parentPath += "/..";
                    if (parentPath.length() > 100) break;
                }
                
                if (!device.vendorId.empty()) {
                    devices.push_back(device);
                }
            }
        }
        
        closedir(dir);
        return devices;
    }

    static std::vector<std::string> findBrainPorts() {
        return findDevicePorts(true);
    }

    static std::vector<std::string> findGPSPorts() {
        return findDevicePorts(false);
    }

private:
    static std::vector<std::string> findDevicePorts(bool isBrain) {
        std::vector<std::string> ports;
        auto devices = listAvailablePorts();
        
        // Group devices by their USB device path
        std::map<std::string, std::vector<DeviceInfo>> deviceGroups;
        for (const auto& device : devices) {
            if ((isBrain && isBrainDevice(device)) || (!isBrain && isGPSDevice(device))) {
                deviceGroups[device.devpath].push_back(device);
            }
        }
        
        // For each device
        for (const auto& group : deviceGroups) {
            auto interfaces = group.second;
            // Find the User Port interface
            auto userPort = std::find_if(interfaces.begin(), interfaces.end(),
                [](const DeviceInfo& d) {
                    return (d.interface.find("User Port") != std::string::npos);
                });
            
            if (userPort != interfaces.end()) {
                ports.push_back(userPort->port);
            }
        }
        
        return ports;
    }

    static bool fileExists(const std::string& path) {
        struct stat buffer;
        return (stat(path.c_str(), &buffer) == 0);
    }

    static std::string readFile(const std::string& path) {
        std::ifstream file(path);
        std::string content;
        if (file) {
            std::getline(file, content);
            content.erase(0, content.find_first_not_of(" \n\r\t"));
            content.erase(content.find_last_not_of(" \n\r\t") + 1);
        }
        return content;
    }

    static bool isBrainDevice(const DeviceInfo& device) {
        return device.vendorId == "2888" && device.productId == "0501";
    }

    static bool isGPSDevice(const DeviceInfo& device) {
        return device.vendorId == "2888" && device.productId == "0521";
    }
};

#endif