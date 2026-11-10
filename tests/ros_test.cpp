#define RC_CAR_NODE_TEST
#include "../src/node.cpp"

#include <cstdlib>
#include <iostream>
#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

using namespace rc_car;
using namespace rc_car::wire;
using namespace std::chrono_literals;
#define CHECK(x) do { if (!(x)) { std::cerr << __LINE__ << ": " #x "\n"; std::abort(); } } while (0)

namespace {
constexpr Text device_id = "1111111111111111", session_id = "2222222222222222";

class FakePico {
public:
    FakePico() {
        char name[128];
        CHECK(openpty(&host_, &device_, name, nullptr, nullptr) == 0);
        path = name;
        CHECK(fcntl(host_, F_SETFL, O_NONBLOCK) == 0);
        state_.session = session_id;
        state_.generation = 1;
        state_.rc = Rc::bad;
        state_.drive_pwm = state_.steer_pwm = state_.pan_pwm = 1500;
        worker_ = std::thread([this] { run(); });
    }
    ~FakePico() {
        stop_ = true;
        worker_.join();
        close(host_);
        close(device_);
    }

    std::string path;
    std::atomic_bool rc_edge{}, drop_next_result{};
    std::atomic_uint mode_count{};

private:
    void send(Line line) { CHECK(::write(host_, line.data.data(), line.size) == ssize_t(line.size)); }
    void on_line(Text line) {
        Fields fields(line);
        if (Hello hello; read(fields, hello)) {
            admitted_ = true;
            send(encode(Identity{hello.challenge, session_id, device_id, "3333333333333333", "RCCAR",
                "waveshare_rp2040_zero", "build", "profile"}));
        } else if (ModeRequest req; read(fields, req)) {
            ++mode_count;
            CHECK(req.key.session == session_id && req.key.tick <= state_.tick);
            if (req.mode != Mode::hold) CHECK(req.generation == state_.generation);
            state_.mode = req.mode;
            state_.drive_choice = req.drive;
            state_.steer_choice = req.steer;
            state_.selected_drive = 0;
            state_.drive_pwm = 1500;
            ++state_.generation;
            if (!drop_next_result.exchange(false))
                send(encode(Result{session_id, req.key.seq, Code::ok, state_.generation}));
        } else if (Drive drive; read(fields, drive)) {
            if (state_.mode != Mode::automatic || drive.generation != state_.generation) return;
            state_.accepted_drive = state_.selected_drive = drive.key.seq;
            state_.drive_pwm = static_cast<uint16_t>(1500 + drive.drive / 20);
        } else if (Pan pan; read(fields, pan)) {
            state_.pan_seq = pan.key.seq;
            state_.pan_pwm = static_cast<uint16_t>(1500 + pan.demand / 20);
        }
    }
    void run() {
        uint64_t last = 0;
        Stream input;
        while (!stop_) {
            const auto now = steady_ms();
            char bytes[512];
            const auto n = ::read(host_, bytes, sizeof(bytes));
            for (ssize_t i = 0; i < n; ++i) {
                const auto record = input.push(bytes[i], now);
                if (!record.empty()) on_line(record);
            }
            if (admitted_ && now - last >= 20) {
                if (rc_edge.exchange(false)) {
                    state_.mode = Mode::neutral;
                    state_.rc = Rc::good;
                    state_.cancel_reason = Reason::rc_returned;
                    state_.cancel_generation = ++state_.generation;
                    state_.selected_drive = 0;
                    state_.drive_pwm = 1500;
                }
                state_.tick = now;
                send(encode(state_));
                last = now;
            }
            std::this_thread::sleep_for(2ms);
        }
    }

    int host_{}, device_{};
    std::atomic_bool stop_{};
    std::thread worker_;
    bool admitted_{};
    Status state_;
};

template<class F> void wait(F predicate) {
    const auto end = std::chrono::steady_clock::now() + 4s;
    while (!predicate() && std::chrono::steady_clock::now() < end) std::this_thread::sleep_for(5ms);
    CHECK(predicate());
}
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    FakePico pico;
    auto vehicle = std::make_shared<VehicleNode>([&] { return UsbDevice{pico.path, std::string(device_id)}; });
    auto test = std::make_shared<rclcpp::Node>("rc_car_test");
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(vehicle);
    executor.add_node(test);
    std::mutex mutex;
    VehicleStatus status;
    const auto observed = [&] { std::lock_guard lock(mutex); return status; };
    const auto qos = rclcpp::QoS(1).best_effort();
    const auto sub = test->create_subscription<VehicleStatus>("/rc_car_interface/status", qos,
        [&](VehicleStatus::ConstSharedPtr msg) { std::lock_guard lock(mutex); status = *msg; });
    const auto drive = test->create_publisher<DriveCommand>("/rc_car_interface/command", qos);
    const auto pan = test->create_publisher<PanCommand>("/rc_car_interface/pan_command", qos);
    const auto mode = test->create_client<SetMode>("/rc_car_interface/set_mode");
    const auto runtime_mode = test->create_client<SetRuntimeMode>("/rc_car_interface/set_runtime_mode");
    std::thread spin([&] { executor.spin(); });
    wait([&] { return observed().fresh && mode->service_is_ready() && runtime_mode->service_is_ready() &&
                      drive->get_subscription_count(); });
    const auto request = [&](uint8_t mode_value) {
        auto req = std::make_shared<SetMode::Request>();
        const auto snap = observed();
        req->stamp = test->now();
        req->session = snap.session;
        req->generation = snap.generation;
        req->mode = mode_value;
        auto result = mode->async_send_request(req);
        CHECK(result.wait_for(2s) == std::future_status::ready);
        return result.get();
    };
    CHECK(request(SetMode::Request::AUTO)->accepted);
    wait([&] { return observed().mode == "AUTO"; });
    DriveCommand target;
    target.session = observed().session;
    target.generation = observed().generation;
    target.stamp = test->now();
    target.drive = .4;
    drive->publish(target);
    wait([&] { return observed().drive_pwm_us == 1700; });
    pico.rc_edge = true;
    wait([&] { return observed().mode == "NEUTRAL" && observed().cancel_reason == "RC_RETURNED"; });
    target.stamp = test->now();
    drive->publish(target); // Old generation stays rejected.
    PanCommand pan_target;
    pan_target.session = target.session;
    pan_target.stamp = test->now();
    pan_target.demand = -.5;
    pan->publish(pan_target);
    wait([&] { return observed().pan_pwm_us == 1250; });
    CHECK(observed().drive_pwm_us == 1500);
    const auto runtime_hold = [&](int64_t offset) {
        auto req = std::make_shared<SetRuntimeMode::Request>();
        const auto snap = observed();
        req->stamp_ns = test->now().nanoseconds() + offset;
        req->session = snap.session;
        req->generation = snap.generation;
        req->mode = SetRuntimeMode::Request::HOLD;
        auto result = runtime_mode->async_send_request(req);
        CHECK(result.wait_for(2s) == std::future_status::ready);
        return result.get();
    };
    CHECK(runtime_hold(-1000000000)->result == "STALE");
    pico.drop_next_result = true;
    const auto hold = runtime_hold(0);
    CHECK(!hold->accepted && hold->result == "UNCERTAIN" && hold->observed_mode == "HOLD");
    CHECK(pico.mode_count == 2);
    executor.cancel();
    spin.join();
    executor.remove_node(vehicle);
    vehicle.reset();
    rclcpp::shutdown();
    std::cout << "ROS mode, drive, RC edge, pan, and dropped result checked over PTY\n";
}
