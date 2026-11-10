#include "rc_car/serial.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

namespace {
using namespace rc_car;
using namespace std::chrono_literals;

struct Options {
    std::string operation, device, profile, build;
    unsigned duration{30};
};

Options parse(int argc, char **argv) {
    Options out;
    if (argc < 2) throw std::invalid_argument("expected status, watch, manual, or hold");
    out.operation = argv[1];
    if (out.operation != "status" && out.operation != "watch" &&
        out.operation != "manual" && out.operation != "hold")
        throw std::invalid_argument("expected status, watch, manual, or hold");
    for (int i = 2; i < argc; i += 2) {
        const wire::Text key{argv[i]};
        if (i + 1 == argc) throw std::invalid_argument("missing option value");
        const std::string value{argv[i + 1]};
        if (key == "--device-id" && wire::hex(value, 16)) out.device = value;
        else if (key == "--profile-id" && wire::label(value)) out.profile = value;
        else if (key == "--build-id" && wire::label(value)) out.build = value;
        else if (key == "--duration" && out.operation == "watch" &&
                 wire::integer(value, out.duration) && out.duration >= 1 && out.duration <= 600) {}
        else throw std::invalid_argument("invalid option or value: " + std::string(key));
    }
    if ((out.operation == "manual" || out.operation == "hold") &&
        (out.device.empty() || out.profile.empty() || out.build.empty()))
        throw std::invalid_argument("mode changes require exact device, profile, and build IDs");
    return out;
}

class Json {
public:
    explicit Json(std::ostream &out) : out_(out) { out_ << '{'; }
    ~Json() { out_ << '}'; }
    void field(wire::Text name, wire::Text value) {
        key(name);
        out_ << std::quoted(std::string(value));
    }
    void number(wire::Text name, uint64_t value) { key(name); out_ << value; }
    void boolean(wire::Text name, bool value) { key(name); out_ << (value ? "true" : "false"); }
    void key(wire::Text name) {
        if (!first_) out_ << ',';
        first_ = false;
        out_ << '"' << name << "\":";
    }
private:
    std::ostream &out_;
    bool first_{true};
};

void status_json(const Observation &view) {
    Json j(std::cout);
    j.boolean("fresh", view.ready);
    j.field("device", view.device);
    j.field("boot", view.boot);
    j.field("board", view.board);
    j.field("build", view.build);
    j.field("profile", view.profile);
    j.field("session", view.session);
    j.number("age_ms", view.age_ms);
    j.number("invalid_frames", view.corrupt);
    if (!view.status) return;
    const auto &s = *view.status;
    j.number("tick_ms", s.tick);
    j.number("generation", s.generation);
    j.field("mode", wire::name(s.mode));
    j.field("reason", wire::name(s.reason));
    j.field("rc_quality", wire::name(s.rc));
    j.number("valid_axes", s.valid_axes);
    j.number("rc_drive_us", s.rc_drive);
    j.number("rc_steer_us", s.rc_steer);
    j.field("drive_choice", wire::name(s.drive_choice));
    j.field("steer_choice", wire::name(s.steer_choice));
    j.field("drive_source", wire::name(s.drive_source));
    j.field("steer_source", wire::name(s.steer_source));
    j.number("saturated_axes", s.saturated);
    j.number("accepted_drive_seq", s.accepted_drive);
    j.number("selected_drive_seq", s.selected_drive);
    j.number("drive_pwm_us", s.drive_pwm);
    j.number("steer_pwm_us", s.steer_pwm);
    j.number("pan_pwm_us", s.pan_pwm);
    j.number("pan_seq", s.pan_seq);
    j.field("cancel_reason", wire::name(s.cancel_reason));
    j.number("cancel_generation", s.cancel_generation);
}

template<class Predicate> bool wait(Link &link, uint64_t limit, Predicate ready) {
    const auto end = steady_ms() + limit;
    while (steady_ms() < end) {
        link.pump(steady_ms());
        if (ready()) return true;
        std::this_thread::sleep_for(4ms);
    }
    return false;
}

int mode(Link &link, wire::Mode requested) {
    auto &session = link.session();
    const auto before = *session.status();
    const auto line = session.mode(steady_ms(), before.generation, requested, wire::Choice::none, wire::Choice::none);
    if (!line.size) throw std::runtime_error("Pico status expired before mode request");
    std::optional<wire::Result> result;
    bool confirmed = false;
    bool interrupted = false;
    try {
        link.send(line);
        confirmed = wait(link, 240, [&] {
            if (!result) result = session.take_result();
            if (!result) return false;
            const auto &now = *session.status();
            if (now.tick <= before.tick || now.generation < result->generation) return false;
            if (result->code != wire::Code::ok) return true;
            return requested == wire::Mode::hold ? now.mode == wire::Mode::hold :
                (now.mode == wire::Mode::manual || now.mode == wire::Mode::neutral);
        });
    } catch (const std::exception &e) {
        // A write or link failure can occur after the device accepts the request.
        interrupted = true;
        std::cerr << "rc_car_ctl: " << e.what() << '\n';
    }
    const bool accepted = result && result->code == wire::Code::ok;
    {
        Json j(std::cout);
        j.field("requested", wire::name(requested));
        j.field("result", result ? wire::name(result->code) : "UNCERTAIN");
        j.boolean("accepted", accepted);
        j.boolean("confirmed", confirmed && accepted);
        if (result) j.number("result_generation", result->generation);
        j.key("observed");
        auto observed = session.observe(steady_ms());
        if (interrupted) observed.ready = false;
        status_json(observed);
    }
    std::cout << std::endl;
    return confirmed && accepted ? 0 : 3;
}

int operate(const Options &o) {
    Link link(select_usb(enumerate_usb(), o.device), o.profile, o.build);
    auto &session = link.session();
    if (!wait(link, 1050, [&] { return session.ready(steady_ms()); }))
        throw std::runtime_error("no fresh Pico status");
    if (o.operation == "manual" || o.operation == "hold")
        return mode(link, o.operation == "manual" ? wire::Mode::manual : wire::Mode::hold);

    status_json(session.observe(steady_ms()));
    std::cout << std::endl;
    if (o.operation == "status") return 0;
    auto tick = session.status()->tick;
    const auto end = steady_ms() + uint64_t(o.duration) * 1000;
    while (steady_ms() < end) {
        const auto now = steady_ms();
        link.pump(now);
        if (session.status()->tick > tick) {
            tick = session.status()->tick;
            status_json(session.observe(now));
            std::cout << std::endl;
        }
        std::this_thread::sleep_for(4ms);
    }
    return 0;
}
}

int main(int argc, char **argv) {
    try {
        return operate(parse(argc, argv));
    } catch (const std::exception &e) {
        std::cerr << "rc_car_ctl: " << e.what() << '\n';
        return 2;
    }
}
