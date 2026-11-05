#pragma once

#include "rc_car/session.hpp"
#include <filesystem>
#include <vector>

namespace rc_car {

struct UsbDevice {
    std::filesystem::path path;
    std::string serial;
};

std::vector<UsbDevice> enumerate_usb();
UsbDevice select_usb(const std::vector<UsbDevice> &devices, const std::string &selector);
uint64_t steady_ms();

class Port {
public:
    explicit Port(const std::filesystem::path &path);
    ~Port();
    Port(const Port &) = delete;
    Port &operator=(const Port &) = delete;

    void read(Session &session, uint64_t now);
    void write(const wire::Line &record);

private:
    int fd_{-1};
};

class Link {
public:
    Link(const UsbDevice &device, const std::string &profile, const std::string &build);
    void pump(uint64_t now);
    Session &session() { return session_; }
    void send(const wire::Line &line) { if (line.size) port_.write(line); }

private:
    Port port_;
    Session session_;
    uint64_t last_pump_{};
};

} // namespace rc_car
