#include "rc_car/serial.hpp"
#include <chrono>
#include <iostream>
#include <poll.h>
#include <thread>
#include <unistd.h>

namespace {
using namespace std::chrono_literals;
using namespace rc_car;

class MaintenanceLink {
public:
    MaintenanceLink(const std::string &device, const std::string &profile, const std::string &build)
        : link_(select_usb(enumerate_usb(), device), profile, build) {}

    void run() {
        auto &session = link_.session();
        pump_until(1000, [&] { return session.ready(steady_ms()); });
        const auto observation = session.observe(steady_ms());
        const auto &status = *observation.status;
        std::cout << "READY " << observation.device << ' ' << observation.profile << ' '
                  << status.drive_pwm << ' ' << status.steer_pwm << std::endl;
        std::string input;
        while (true) {
            link_.pump(steady_ms());
            pollfd p{STDIN_FILENO, POLLIN, 0};
            if (::poll(&p, 1, 5) < 0) {
                if (errno == EINTR) continue;
                throw std::runtime_error("maintenance input poll failed");
            }
            if (p.revents & (POLLHUP | POLLERR | POLLNVAL)) return;
            if (!(p.revents & POLLIN)) continue;
            char bytes[64];
            const auto n = ::read(STDIN_FILENO, bytes, sizeof(bytes));
            if (n <= 0) return;
            for (ssize_t i = 0; i < n; ++i) {
                if (bytes[i] == '\n') {
                    if (command(input)) return;
                    input.clear();
                } else {
                    if (input.size() >= 32) throw std::runtime_error("maintenance input exceeds bound");
                    input.push_back(bytes[i]);
                }
            }
        }
    }

private:
    template<class Predicate> void pump_until(uint64_t timeout, Predicate ready) {
        const auto end = steady_ms() + timeout;
        while (steady_ms() < end) {
            link_.pump(steady_ms());
            if (ready()) return;
            std::this_thread::sleep_for(3ms);
        }
        throw std::runtime_error("maintenance response deadline");
    }

    bool command(wire::Text text) {
        auto &session = link_.session();
        const auto now = steady_ms();
        if (!session.ready(now)) throw std::runtime_error("no fresh Pico status");
        const auto tick = session.status()->tick;
        wire::Mode want = wire::Mode::maintenance;
        wire::Line record;
        std::optional<uint16_t> pulse;
        bool done = false;
        if (text == "HOLD") {
            want = wire::Mode::hold;
            record = session.mode(now, session.status()->generation, want, wire::Choice::none, wire::Choice::none);
        } else if (text == "ENTER") {
            record = session.maintenance(now, wire::Operation::enter);
        } else if (text == "ABORT" || text == "EXIT") {
            want = wire::Mode::hold;
            record = session.maintenance(now, text == "ABORT" ? wire::Operation::abort : wire::Operation::exit);
            done = true;
        } else if (text.substr(0, 6) == "PULSE ") {
            pulse.emplace();
            if (!wire::integer(text.substr(6), *pulse)) throw std::invalid_argument("invalid maintenance pulse");
            record = session.maintenance(now, wire::Operation::pulse, *pulse);
        } else {
            throw std::invalid_argument("unknown maintenance command");
        }
        link_.send(record);
        std::optional<wire::Result> result;
        pump_until(120, [&] {
            if (!result) result = session.take_result();
            if (!result) return false;
            if (result->code != wire::Code::ok)
                throw std::runtime_error("Pico rejected maintenance command: " + std::string(wire::name(result->code)));
            const auto &status = *session.status();
            return status.tick > tick && status.generation >= result->generation && status.mode == want &&
                (!pulse || status.drive_pwm == *pulse);
        });
        const auto &status = *session.status();
        std::cout << "OK " << wire::name(status.mode) << ' ' << status.drive_pwm << ' '
                  << status.steer_pwm << std::endl;
        return done;
    }

    Link link_;
};
}

int main(int argc, char **argv) {
    if (argc != 3 && argc != 4) {
        std::cerr << "Usage: rc_car_maintenance_link DEVICE_ID PROFILE_ID [BUILD_ID]\n";
        return 2;
    }
    try {
        if (!wire::hex(argv[1], 16) || !wire::label(argv[2]) || (argc == 4 && !wire::label(argv[3])))
            throw std::invalid_argument("an exact device and profile are required");
        MaintenanceLink(argv[1], argv[2], argc == 4 ? argv[3] : "").run();
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "rc_car_maintenance_link: " << e.what() << '\n';
        return 1;
    }
}
