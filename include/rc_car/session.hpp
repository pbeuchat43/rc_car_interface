#pragma once

#include "rc_car/wire.hpp"
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace rc_car {

struct Expected {
    std::string device, profile, build;
};
struct Observation {
    std::string session, device, boot, board, build, profile;
    std::optional<wire::Status> status;
    bool ready{};
    uint64_t age_ms{};
    unsigned corrupt{};
};

inline std::optional<int> quantize(double value) {
    if (!std::isfinite(value) || value < -1.0 || value > 1.0) return {};
    return static_cast<int>(std::lround(value * wire::full));
}

// ROS stamps are system-clock nanoseconds. Lease timers use the steady clock.
inline std::optional<uint64_t> deadline(int64_t stamp, int64_t system_now, uint64_t steady_now) {
    constexpr int64_t future_ns = 20'000'000, max_age_ns = 200'000'000;
    if (stamp <= 0 || system_now <= 0 || stamp - system_now > future_ns) return {};
    const int64_t age = stamp < system_now ? system_now - stamp : 0;
    if (age >= max_age_ns) return {};
    const auto remaining_ms = static_cast<uint64_t>((max_age_ns - age) / 1'000'000);
    if (!remaining_ms) return {};
    return steady_now + remaining_ms;
}

class Session {
public:
    Session(Expected expected, std::string challenge, uint64_t opened)
        : expected_(std::move(expected)), challenge_(std::move(challenge)), opened_(opened) {
        if (!wire::hex(expected_.device, 16) || !wire::hex(challenge_, 16) ||
            (!expected_.profile.empty() && !wire::label(expected_.profile)) ||
            (!expected_.build.empty() && !wire::label(expected_.build)))
            throw std::invalid_argument("invalid RC Car identity configuration");
    }

    wire::Line hello(uint64_t now) {
        if (!view_.session.empty() || (hello_at_ && now - *hello_at_ < 100)) return {};
        hello_at_ = now;
        return wire::encode(wire::Hello{challenge_});
    }

    void push(char byte, uint64_t now) {
        const auto record = stream_.push(byte, now);
        if (record.empty()) return;
        wire::Fields fields(record);
        wire::Identity identity;
        wire::Status status;
        wire::Result result;
        if (wire::read(fields, identity)) {
            if (identity.challenge != challenge_) return;
            if (identity.role != "RCCAR" || identity.device != expected_.device ||
                (!expected_.profile.empty() && identity.profile != expected_.profile) ||
                (!expected_.build.empty() && identity.build != expected_.build))
                throw std::runtime_error("Pico identity, profile, or build mismatch");
            if (identity.board != "waveshare_rp2040_zero" && identity.board != "waveshare_rp2350_zero")
                throw std::runtime_error("unsupported Pico board");
            if (!view_.session.empty() && (view_.session != identity.session || view_.boot != identity.boot))
                throw std::runtime_error("Pico session changed");
            view_.session = identity.session;
            view_.device = identity.device;
            view_.boot = identity.boot;
            view_.board = identity.board;
            view_.build = identity.build;
            view_.profile = identity.profile;
            return;
        }
        if (wire::read(fields, status)) {
            if (status.session != view_.session || (view_.status && status.tick <= view_.status->tick)) return;
            if (view_.status && status.generation < view_.status->generation)
                throw std::runtime_error("Pico authority generation regressed");
            status.session = {};
            view_.status = status;
            received_ = now;
            return;
        }
        if (wire::read(fields, result)) {
            if (result.session != view_.session || result.seq != waiting_for_) return;
            result.session = {};
            result_ = result;
            return;
        }
        ++view_.corrupt;
    }

    bool expired(uint64_t now) const {
        return view_.status ? now - received_ >= wire::status_timeout_ms : now - opened_ >= 1000;
    }

    bool ready(uint64_t now) const { return view_.status && !expired(now); }
    const std::optional<wire::Status> &status() const { return view_.status; }
    const std::string &token() const { return view_.session; }
    Observation observe(uint64_t now) const {
        auto out = view_;
        out.ready = ready(now);
        if (out.status) out.age_ms = now - received_;
        return out;
    }

    wire::Line mode(uint64_t now, uint64_t generation, wire::Mode requested,
                    wire::Choice drive, wire::Choice steer) {
        if (requested == wire::Mode::automatic && drive == wire::Choice::rc && steer == wire::Choice::rc) {
            requested = wire::Mode::manual;
            drive = steer = wire::Choice::none;
        }
        if (!ready(now)) return {};
        const auto key = mutation();
        expect(key.seq);
        return wire::encode(wire::ModeRequest{key, generation, requested, drive, steer});
    }

    wire::Line drive(uint64_t now, uint64_t generation, int d, int s) {
        if (!ready(now) || view_.status->generation != generation || view_.status->mode != wire::Mode::automatic ||
            !wire::demand(d) || !wire::demand(s)) return {};
        return wire::encode(wire::Drive{mutation(), generation, d, s});
    }

    wire::Line pan(uint64_t now, int demand) {
        if (!ready(now) || view_.status->mode == wire::Mode::maintenance || !wire::demand(demand)) return {};
        return wire::encode(wire::Pan{mutation(), demand});
    }

    wire::Line maintenance(uint64_t now, wire::Operation op, uint16_t pulse = 0) {
        if (!ready(now)) return {};
        const auto key = mutation();
        expect(key.seq);
        return wire::encode(wire::Maintenance{key, view_.status->generation, op, pulse});
    }

    std::optional<wire::Result> take_result() {
        if (!result_) return {};
        waiting_for_ = 0;
        return std::exchange(result_, {});
    }

private:
    wire::Mutation mutation() {
        if (seq_ == UINT32_MAX) throw std::runtime_error("RCPv1 sequence exhausted");
        return {view_.session, ++seq_, view_.status->tick};
    }
    void expect(uint32_t seq) { waiting_for_ = seq; result_.reset(); }

    Expected expected_;
    std::string challenge_;
    Observation view_;
    uint64_t opened_, received_{};
    std::optional<uint64_t> hello_at_;
    wire::Stream stream_;
    uint32_t seq_{}, waiting_for_{};
    std::optional<wire::Result> result_;
};

} // namespace rc_car
