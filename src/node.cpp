#include "rc_car/serial.hpp"
#include "rc_car_interface/msg/drive_command.hpp"
#include "rc_car_interface/msg/pan_command.hpp"
#include "rc_car_interface/msg/vehicle_status.hpp"
#include "rc_car_interface/srv/set_mode.hpp"
#include "rc_car_interface/srv/set_runtime_mode.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace rc_car {
namespace {
using Steady = std::chrono::steady_clock;
using System = std::chrono::system_clock;
using namespace std::chrono_literals;

int64_t system_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(System::now().time_since_epoch()).count();
}

int64_t ns(const builtin_interfaces::msg::Time &time) {
    return time.nanosec < 1'000'000'000 ? int64_t(time.sec) * 1'000'000'000 + time.nanosec : 0;
}

builtin_interfaces::msg::Time stamp() {
    return rclcpp::Time(system_ns(), RCL_SYSTEM_TIME);
}

template<class E> std::string token(E e) { return std::string(wire::name(e)); }

using DriveCommand = rc_car_interface::msg::DriveCommand;
using PanCommand = rc_car_interface::msg::PanCommand;
using VehicleStatus = rc_car_interface::msg::VehicleStatus;
using SetMode = rc_car_interface::srv::SetMode;
using SetRuntimeMode = rc_car_interface::srv::SetRuntimeMode;

struct Targets {
    uint64_t generation, until;
    int drive, steer;
};
struct PanTarget {
    uint64_t until;
    int demand;
};
struct Request {
    wire::Mode mode;
    wire::Choice drive, steer;
    uint64_t generation, until;
    bool sent{};
    std::string result;
    uint64_t resulting_generation{};
};
class VehicleNode : public rclcpp::Node {
public:
    explicit VehicleNode(std::function<UsbDevice()> discover = {})
        : Node("rc_car_interface"), discover_(std::move(discover)) {
        rcl_interfaces::msg::ParameterDescriptor read_only;
        read_only.read_only = true;
        device_ = declare_parameter<std::string>("device_id", "", read_only);
        profile_ = declare_parameter<std::string>("profile_id", "", read_only);
        build_ = declare_parameter<std::string>("build_id", "", read_only);
        if ((!device_.empty() && !wire::hex(device_, 16)) ||
            (!profile_.empty() && !wire::label(profile_)) || (!build_.empty() && !wire::label(build_)))
            throw std::invalid_argument("invalid device_id, profile_id or build_id");
        if (get_parameter("use_sim_time").as_bool()) throw std::invalid_argument("physical actuation requires system time");
        parameters_ = add_on_set_parameters_callback([](const auto &params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto &p : params) {
                if (p.get_name() == "use_sim_time" && p.as_bool()) {
                    result.successful = false;
                    result.reason = "physical actuation requires system time";
                }
            }
            return result;
        });
        const auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
        drive_sub_ = create_subscription<DriveCommand>("~/command", qos, [this](DriveCommand::ConstSharedPtr msg) {
            const auto d = quantize(msg->drive), s = quantize(msg->steer);
            const auto until = deadline(ns(msg->stamp), system_ns(), steady_ms());
            std::lock_guard lock(mutex_);
            if (!d || !s || !until || !live() || msg->session != observation_.session ||
                msg->generation != observation_.status->generation || observation_.status->mode != wire::Mode::automatic) {
                ++rejected_;
                return;
            }
            target_ = Targets{msg->generation, *until, *d, *s};
        });
        pan_sub_ = create_subscription<PanCommand>("~/pan_command", qos, [this](PanCommand::ConstSharedPtr msg) {
            const auto demand = quantize(msg->demand);
            const auto until = deadline(ns(msg->stamp), system_ns(), steady_ms());
            std::lock_guard lock(mutex_);
            if (!demand || !until || !live() || msg->session != observation_.session ||
                observation_.status->mode == wire::Mode::maintenance) {
                ++rejected_;
                return;
            }
            pan_ = PanTarget{*until, *demand};
        });
        mode_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        mode_service_ = create_service<SetMode>("~/set_mode",
            [this](const SetMode::Request::SharedPtr req, SetMode::Response::SharedPtr res) {
                mode(ns(req->stamp), *req, *res);
            },
            rclcpp::ServicesQoS(), mode_group_);
        runtime_mode_service_ = create_service<SetRuntimeMode>("~/set_runtime_mode",
            [this](const SetRuntimeMode::Request::SharedPtr req, SetRuntimeMode::Response::SharedPtr res) {
                mode(req->stamp_ns, *req, *res);
            },
            rclcpp::ServicesQoS(), mode_group_);
        status_pub_ = create_publisher<VehicleStatus>("~/status", qos);
        diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 1);
        status_timer_ = create_wall_timer(40ms, [this] { publish(); });
        diagnostic_timer_ = create_wall_timer(1s, [this] { diagnostics(); });
        worker_ = std::thread([this] { run(); });
    }

    ~VehicleNode() override {
        stopping_ = true;
        wake_.notify_all();
        if (worker_.joinable()) worker_.join();
    }

private:
    bool live() const { return observation_.ready && observation_.status.has_value(); }

    template<class RequestMessage, class ResponseMessage>
    void mode(int64_t stamp_ns, const RequestMessage &req, ResponseMessage &response) {
        const auto now = steady_ms();
        const auto until = deadline(stamp_ns, system_ns(), now);
        std::unique_lock lock(mutex_);
        if (!until) response.result = "STALE";
        else if (!live() || req.session != observation_.session) response.result = "UNAVAILABLE";
        else if (req.mode < SetMode::Request::MANUAL || req.mode > SetMode::Request::HOLD ||
                 req.drive_choice > SetMode::Request::SUM || req.steer_choice > SetMode::Request::SUM ||
                 (req.mode != SetMode::Request::AUTO && (req.drive_choice || req.steer_choice))) response.result = "BAD_RANGE";
        else if (req.mode != SetMode::Request::HOLD && req.generation != observation_.status->generation)
            response.result = "WRONG_GENERATION";
        else if (request_ && req.mode != SetMode::Request::HOLD) response.result = "BUSY";
        else {
            if (request_) finish(request_->sent ? "UNCERTAIN" : "SUPERSEDED");
            auto d = static_cast<wire::Choice>(req.drive_choice);
            auto s = static_cast<wire::Choice>(req.steer_choice);
            if (req.mode == SetMode::Request::AUTO) {
                if (d == wire::Choice::none) d = wire::Choice::host;
                if (s == wire::Choice::none) s = wire::Choice::host;
            }
            const auto op = std::make_shared<Request>(Request{
                static_cast<wire::Mode>(req.mode), d, s, req.generation, *until, false, {}, 0
            });
            request_ = op;
            wake_.wait_until(lock, Steady::now() + 250ms, [&] { return stopping_ || !op->result.empty(); });
            if (op->result.empty() && request_ == op) finish(op->sent ? "UNCERTAIN" : "UNAVAILABLE");
            response.result = op->result;
            response.accepted = op->result == "OK";
            response.resulting_generation = op->resulting_generation;
        }
        response.observed_fresh = live();
        if (live()) {
            response.observed_mode = token(observation_.status->mode);
            response.observed_generation = observation_.status->generation;
        }
    }

    void finish(const std::string &result, uint64_t generation = 0) {
        if (!request_) return;
        request_->result = result;
        request_->resulting_generation = generation;
        request_.reset();
        wake_.notify_all();
    }

    void invalidate(const std::string &reason) {
        std::lock_guard lock(mutex_);
        target_.reset();
        pan_.reset();
        observation_.ready = false;
        observation_.session.clear();
        finish(request_ && request_->sent ? "UNCERTAIN" : "UNAVAILABLE");
        connection_ = reason;
    }

    void transmit(Link &link, uint64_t now, uint64_t &last_drive) {
        auto &session = link.session();
        auto latest = session.observe(now);
        const auto result = session.take_result();
        // Hold the mutex through each bounded write so a superseded request
        // cannot be transmitted after a competing Hold has been admitted.
        std::lock_guard lock(mutex_);
        if (latest.session != observation_.session) {
            target_.reset();
            pan_.reset();
        } else if (target_ && (!latest.status || target_->generation != latest.status->generation)) {
            target_.reset();
        }
        observation_ = std::move(latest);
        connection_ = observation_.ready ? "ready" : "negotiating";
        if (request_ && request_->sent && result) finish(token(result->code), result->generation);
        if (!live()) return;
        if (request_ && !request_->sent) {
            if (now >= request_->until) finish("STALE");
            else {
                const auto record = session.mode(now, request_->generation, request_->mode, request_->drive, request_->steer);
                if (!record.size) finish("UNAVAILABLE");
                else {
                    request_->sent = true;
                    // Pause drive until a mode decision arrives or times out.
                    target_.reset();
                    link.send(record);
                    return;
                }
            }
        }
        if (pan_) {
            if (steady_ms() < pan_->until) link.send(session.pan(now, pan_->demand));
            pan_.reset();
        }
        if (!request_ && target_ && now - last_drive >= 20) {
            if (steady_ms() >= target_->until) target_.reset();
            else {
                link.send(session.drive(now, target_->generation, target_->drive, target_->steer));
                last_drive = now;
            }
        }
    }

    void run() {
        std::unique_ptr<Link> link;
        uint64_t retry = 0, last_drive = 0;
        int64_t offset = 0;
        while (!stopping_) {
            const auto now = steady_ms();
            const auto new_offset = system_ns() - int64_t(now) * 1'000'000;
            try {
                if (offset && std::abs(new_offset - offset) >= 100'000'000)
                    throw std::runtime_error("system clock discontinuity");
                if (!link && now >= retry) {
                    const auto device = discover_ ? discover_() : select_usb(enumerate_usb(), device_);
                    link = std::make_unique<Link>(device, profile_, build_);
                    last_drive = 0;
                }
                if (link) {
                    const auto tick = steady_ms();
                    link->pump(tick);
                    transmit(*link, tick, last_drive);
                }
            } catch (const std::exception &e) {
                link.reset();
                retry = steady_ms() + 500;
                invalidate(e.what());
            }
            offset = new_offset;
            std::this_thread::sleep_for(5ms);
        }
        link.reset();
        invalidate("shutdown");
    }

    void publish() {
        VehicleStatus msg;
        msg.stamp = stamp();
        {
            std::lock_guard lock(mutex_);
            msg.connected = !observation_.session.empty();
            msg.fresh = live();
            msg.connection = connection_;
            msg.session = observation_.session;
            msg.device = observation_.device;
            msg.boot = observation_.boot;
            msg.board = observation_.board;
            msg.build = observation_.build;
            msg.profile = observation_.profile;
            msg.status_age_ms = observation_.age_ms;
            if (observation_.status) {
                const auto &s = *observation_.status;
                msg.tick_ms = s.tick;
                msg.generation = s.generation;
                msg.mode = token(s.mode);
                msg.reason = token(s.reason);
                msg.cancel_generation = s.cancel_generation;
                msg.cancel_reason = token(s.cancel_reason);
                msg.rc_quality = token(s.rc);
                msg.valid_axes = s.valid_axes;
                msg.rc_drive_us = s.rc_drive;
                msg.rc_steer_us = s.rc_steer;
                msg.drive_choice = token(s.drive_choice);
                msg.steer_choice = token(s.steer_choice);
                msg.drive_source = token(s.drive_source);
                msg.steer_source = token(s.steer_source);
                msg.saturated_axes = s.saturated;
                msg.accepted_drive_seq = s.accepted_drive;
                msg.selected_drive_seq = s.selected_drive;
                msg.drive_pwm_us = s.drive_pwm;
                msg.steer_pwm_us = s.steer_pwm;
                msg.pan_seq = s.pan_seq;
                msg.pan_pwm_us = s.pan_pwm;
            }
        }
        status_pub_->publish(msg);
    }

    void diagnostics() {
        diagnostic_msgs::msg::DiagnosticArray msg;
        msg.header.stamp = stamp();
        diagnostic_msgs::msg::DiagnosticStatus diag;
        diag.name = get_fully_qualified_name();
        {
            std::lock_guard lock(mutex_);
            diag.level = live() ? diag.OK : diag.STALE;
            diag.message = connection_;
            diag.hardware_id = observation_.device;
            diagnostic_msgs::msg::KeyValue corrupt, rejected;
            corrupt.key = "invalid_frames";
            corrupt.value = std::to_string(observation_.corrupt);
            rejected.key = "rejected_inputs";
            rejected.value = std::to_string(rejected_);
            diag.values = {corrupt, rejected};
        }
        msg.status.push_back(diag);
        diagnostics_pub_->publish(msg);
    }

    std::string device_, profile_, build_;
    std::function<UsbDevice()> discover_;
    std::mutex mutex_;
    std::condition_variable wake_;
    std::atomic_bool stopping_{false};
    std::thread worker_;
    Observation observation_;
    std::optional<Targets> target_;
    std::optional<PanTarget> pan_;
    std::shared_ptr<Request> request_;
    std::string connection_{"starting"};
    uint64_t rejected_{};
    rclcpp::CallbackGroup::SharedPtr mode_group_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_;
    rclcpp::Subscription<DriveCommand>::SharedPtr drive_sub_;
    rclcpp::Subscription<PanCommand>::SharedPtr pan_sub_;
    rclcpp::Service<SetMode>::SharedPtr mode_service_;
    rclcpp::Service<SetRuntimeMode>::SharedPtr runtime_mode_service_;
    rclcpp::Publisher<VehicleStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_, diagnostic_timer_;
};

} // namespace
} // namespace rc_car

#ifndef RC_CAR_NODE_TEST
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
        auto node = std::make_shared<rc_car::VehicleNode>();
        executor.add_node(node);
        executor.spin();
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rc_car_interface"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
}
#endif
