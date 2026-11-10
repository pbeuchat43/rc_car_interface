#include "rc_car/serial.hpp"
#include <chrono>
#include <fstream>
#include <random>
#include <system_error>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace rc_car {
namespace fs = std::filesystem;

namespace {
std::string attribute(const fs::path &dir, const char *name) {
    std::ifstream in(dir / name);
    std::string value;
    if (in) std::getline(in, value);
    return value;
}

[[noreturn]] void fail(const char *operation) {
    throw std::system_error(errno, std::system_category(), operation);
}

std::string challenge() {
    std::random_device random;
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%08X%08X", unsigned(random()), unsigned(random()));
    return buf;
}
}

std::vector<UsbDevice> enumerate_usb() {
    std::vector<UsbDevice> devices;
    for (const auto &entry : fs::directory_iterator("/sys/class/tty")) {
        const auto name = entry.path().filename().string();
        if (name.compare(0, 6, "ttyACM") != 0) continue;
        std::error_code error;
        auto parent = fs::canonical(entry.path() / "device", error);
        if (error) continue;
        while (parent != parent.root_path()) {
            const auto vid = attribute(parent, "idVendor");
            if (!vid.empty()) {
                const auto pid = attribute(parent, "idProduct");
                const auto serial = attribute(parent, "serial");
                if (vid == "2e8a" && (pid == "000a" || pid == "0009") &&
                    attribute(parent, "product") == "RC Car Pico" && wire::hex(serial, 16))
                    devices.push_back({fs::path("/dev") / name, serial});
                break;
            }
            parent = parent.parent_path();
        }
    }
    return devices;
}

UsbDevice select_usb(const std::vector<UsbDevice> &devices, const std::string &selector) {
    std::optional<UsbDevice> selected;
    for (const auto &device : devices) {
        if (!selector.empty() && selector != device.serial) continue;
        if (selected) throw std::runtime_error("multiple matching RC Car Picos");
        selected = device;
    }
    if (!selected) throw std::runtime_error("RC Car Pico unavailable");
    return *selected;
}

uint64_t steady_ms() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

Port::Port(const fs::path &path) {
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) fail("open RC Car serial");
    try {
        if (flock(fd_, LOCK_EX | LOCK_NB) < 0 || ioctl(fd_, TIOCEXCL) < 0) fail("claim RC Car serial");
        termios cfg{};
        if (tcgetattr(fd_, &cfg) < 0) fail("read RC Car termios");
        cfmakeraw(&cfg);
        cfg.c_cflag |= CLOCAL | CREAD | HUPCL;
        cfg.c_cflag &= ~CRTSCTS;
        cfg.c_cc[VMIN] = 1;
        cfg.c_cc[VTIME] = 0;
        if (cfsetspeed(&cfg, B115200) < 0 || tcsetattr(fd_, TCSANOW, &cfg) < 0 ||
            tcflush(fd_, TCIOFLUSH) < 0) fail("set RC Car termios");
    } catch (...) {
        ::close(fd_);
        throw;
    }
}

Port::~Port() {
    // Close with HUPCL. Dropping an unsent tail cannot become a valid RCPv1 frame.
    tcflush(fd_, TCIOFLUSH);
    ::close(fd_);
}

void Port::read(Session &session, uint64_t now) {
    char bytes[1024];
    const auto size = ::read(fd_, bytes, sizeof(bytes));
    if (size < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return;
        fail("read RC Car serial");
    }
    if (size == 0) throw std::runtime_error("RC Car serial disconnected");
    for (ssize_t i = 0; i < size; ++i) session.push(bytes[i], now);
}

void Port::write(const wire::Line &record) {
    std::size_t offset = 0;
    const auto end = steady_ms() + 5;
    while (offset < record.size) {
        const auto size = ::write(fd_, record.data.data() + offset, record.size - offset);
        if (size > 0) offset += static_cast<std::size_t>(size);
        else if (size < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
            fail("write RC Car serial");
        if (offset == record.size) break;
        const auto now = steady_ms();
        if (now >= end) throw std::runtime_error("RC Car serial write deadline");
        pollfd poll_target{fd_, POLLOUT, 0};
        if (::poll(&poll_target, 1, static_cast<int>(end - now)) < 0 && errno != EINTR)
            fail("poll RC Car serial");
    }
}

Link::Link(const UsbDevice &device, const std::string &profile, const std::string &build)
    : port_(device.path), session_({device.serial, profile, build}, challenge(), steady_ms()) {}

void Link::pump(uint64_t now) {
    if (last_pump_ && now - last_pump_ >= wire::frame_timeout_ms)
        throw std::runtime_error("RC Car transport service gap");
    last_pump_ = now;
    if (session_.expired(now)) throw std::runtime_error("RC Car identity or status timeout");
    port_.read(session_, now);
    send(session_.hello(now));
}

} // namespace rc_car
