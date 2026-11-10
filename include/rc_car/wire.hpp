#pragma once

#include <array>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string_view>
#include <type_traits>

namespace rc_car::wire {

using Text = std::string_view;
constexpr Text frame_marker = "@RCPv1";
constexpr std::size_t max_frame = 256;
constexpr int full = 10000;
constexpr uint64_t frame_timeout_ms = 100;
constexpr uint64_t status_timeout_ms = 150;
constexpr uint64_t status_period_ms = 40;

enum class Mode { neutral, manual, automatic, hold, maintenance };
enum class Choice { none, host, rc, sum };
enum class Rc { unknown, good, bad };
enum class Reason { none, rc_unknown, rc_bad, wait_neutral, wait_drive, rc_lost, rc_returned,
                    drive_timeout, link_lost, requested, fault, maintenance_timeout, maintenance_abort, new_choices };
enum class Code { ok, wrong_generation, stale, conflict, bad_range, bad_sequence };
enum class Operation { enter, pulse, abort, exit };

constexpr std::array mode_names{"NEUTRAL", "MANUAL", "AUTO", "HOLD", "MAINTENANCE"};
constexpr std::array choice_names{"NONE", "HOST", "RC", "SUM"};
constexpr std::array rc_names{"UNKNOWN", "GOOD", "BAD"};
constexpr std::array reason_names{"NONE", "RC_UNKNOWN", "RC_BAD", "WAIT_NEUTRAL", "WAIT_DRIVE", "RC_LOST",
    "RC_RETURNED", "DRIVE_TIMEOUT", "LINK_LOST", "REQUESTED", "FAULT", "MAINTENANCE_TIMEOUT", "MAINTENANCE_ABORT", "NEW_CHOICES"};
constexpr std::array code_names{"OK", "WRONG_GENERATION", "STALE", "CONFLICT", "BAD_RANGE", "BAD_SEQUENCE"};
constexpr std::array operation_names{"ENTER", "PULSE", "ABORT", "EXIT"};
constexpr auto &names(Mode) { return mode_names; }
constexpr auto &names(Choice) { return choice_names; }
constexpr auto &names(Rc) { return rc_names; }
constexpr auto &names(Reason) { return reason_names; }
constexpr auto &names(Code) { return code_names; }
constexpr auto &names(Operation) { return operation_names; }

template<class E> constexpr Text name(E value) {
    return names(value)[static_cast<unsigned>(value)];
}

struct Hello { Text challenge; };
struct Identity { Text challenge, session, device, boot, role, board, build, profile; };
struct Mutation { Text session; uint32_t seq{}; uint64_t tick{}; };
struct ModeRequest {
    Mutation key;
    uint64_t generation{};
    Mode mode{};
    Choice drive{}, steer{};
};
struct Drive { Mutation key; uint64_t generation{}; int drive{}, steer{}; };
struct Pan { Mutation key; int demand{}; };
struct Maintenance { Mutation key; uint64_t generation{}; Operation operation{}; uint16_t pulse{}; };
struct Result { Text session; uint32_t seq{}; Code code{}; uint64_t generation{}; };
struct Status {
    Text session;
    uint64_t tick{}, generation{};
    Mode mode{};
    Reason reason{};
    uint64_t cancel_generation{};
    Reason cancel_reason{};
    Rc rc{};
    uint8_t valid_axes{};
    uint16_t rc_drive{}, rc_steer{};
    Choice drive_choice{}, steer_choice{}, drive_source{}, steer_source{};
    uint8_t saturated{};
    uint32_t accepted_drive{}, selected_drive{};
    uint16_t drive_pwm{}, steer_pwm{};
    uint32_t pan_seq{};
    uint16_t pan_pwm{};
};

inline uint32_t crc32(Text bytes) {
    uint32_t result = 0xffffffffu;
    for (unsigned char byte : bytes) {
        result ^= byte;
        for (int bit = 0; bit < 8; ++bit)
            result = (result >> 1) ^ ((result & 1) ? 0xedb88320u : 0u);
    }
    return result ^ 0xffffffffu;
}

inline bool hex(Text text, std::size_t size) {
    if (text.size() != size) return false;
    for (const char c : text)
        if (!(c >= '0' && c <= '9') && !(c >= 'A' && c <= 'F')) return false;
    return true;
}

inline bool label(Text text) {
    if (text.empty() || text.size() > 24) return false;
    for (const char c : text)
        if (!(c >= '0' && c <= '9') && !(c >= 'A' && c <= 'Z') &&
            !(c >= 'a' && c <= 'z') && c != '-' && c != '_') return false;
    return true;
}

template<class I> bool integer(Text text, I &out) {
    static_assert(std::is_integral_v<I>);
    if (text.empty()) return false;
    bool negative = false;
    if constexpr (std::is_signed_v<I>) {
        if (text.front() == '-') {
            text.remove_prefix(1);
            negative = true;
        }
    }
    if (text.empty() || (text.front() == '0' && (negative || text.size() != 1))) return false;
    uint64_t value = 0;
    const uint64_t limit = uint64_t(std::numeric_limits<I>::max()) + unsigned(negative);
    for (char c : text) {
        if (c < '0' || c > '9') return false;
        const unsigned digit = c - '0';
        if (value > (limit - digit) / 10) return false;
        value = value * 10 + digit;
    }
    if constexpr (std::is_signed_v<I>) {
        if (negative) {
            out = value == limit ? std::numeric_limits<I>::min() : -static_cast<I>(value);
            return true;
        }
    }
    out = static_cast<I>(value);
    return true;
}

class Stream {
public:
    void reset() { used_ = 0; discarding_ = false; }
    void expire(uint64_t now) {
        if (used_ && now - started_ >= frame_timeout_ms) {
            used_ = 0;
            discarding_ = true;
        }
    }
    Text push(char byte, uint64_t now) {
        expire(now);
        if (discarding_) {
            if (byte == '\n') reset();
            return {};
        }
        if (byte == '\n') {
            const Text record{bytes_.data(), used_};
            used_ = 0;
            return record;
        }
        if (byte < '!' || byte > '~' || used_ == max_frame - 1) {
            used_ = 0;
            discarding_ = true;
            return {};
        }
        if (used_ == 0) started_ = now;
        bytes_[used_++] = byte;
        return {};
    }

private:
    std::array<char, max_frame> bytes_{};
    std::size_t used_{};
    uint64_t started_{};
    bool discarding_{};
};

class Fields {
public:
    explicit Fields(Text line) {
        constexpr auto fields_begin = frame_marker.size() + 1;
        if (line.size() <= fields_begin || line.size() >= max_frame ||
            line.substr(0, frame_marker.size()) != frame_marker || line[frame_marker.size()] != ',') return;
        const auto comma = line.rfind(',');
        if (comma < fields_begin) return;
        const auto checksum = line.substr(comma + 1);
        if (!hex(checksum, 8)) return;
        uint32_t expected = 0;
        for (const char c : checksum) expected = expected * 16 + (c <= '9' ? c - '0' : c - 'A' + 10);
        if (crc32(line.substr(0, comma + 1)) != expected) return;
        fields_ = line.substr(fields_begin, comma + 1 - fields_begin);
        type_ = take();
    }
    Text type() const { return type_; }
    bool done() const { return ok_ && fields_.empty(); }
    Text take() {
        const auto comma = fields_.find(',');
        if (comma == Text::npos || comma == 0) {
            ok_ = false;
            return {};
        }
        const auto result = fields_.substr(0, comma);
        fields_.remove_prefix(comma + 1);
        return result;
    }
    bool id(Text &value) { value = take(); return hex(value, 16); }
    bool tag(Text &value) { value = take(); return label(value); }
    template<class I> bool number(I &value) { return integer(take(), value); }
    template<class E> bool enumeration(E &value) {
        const auto text = take();
        const auto &table = names(value);
        for (unsigned i = 0; i < table.size(); ++i) {
            if (text == table[i]) {
                value = static_cast<E>(i);
                return true;
            }
        }
        return false;
    }

private:
    Text fields_, type_;
    bool ok_{true};
};

inline bool prefix(Fields &f, Mutation &m) { return f.id(m.session) && f.number(m.seq) && m.seq != 0; }

inline bool read(Fields &f, Hello &x) { return f.type() == "HELLO" && f.id(x.challenge) && f.done(); }

inline bool read(Fields &f, Identity &x) {
    return f.type() == "ID" && f.id(x.challenge) && f.id(x.session) && f.id(x.device) &&
        f.id(x.boot) && f.tag(x.role) && f.tag(x.board) && f.tag(x.build) && f.tag(x.profile) && f.done();
}

inline bool read(Fields &f, ModeRequest &x) {
    return f.type() == "MODE" && prefix(f, x.key) && f.number(x.generation) && f.number(x.key.tick) &&
        f.enumeration(x.mode) && f.enumeration(x.drive) && f.enumeration(x.steer) && f.done();
}

inline bool read(Fields &f, Drive &x) {
    return f.type() == "DRIVE" && prefix(f, x.key) && f.number(x.generation) && f.number(x.key.tick) &&
        f.number(x.drive) && f.number(x.steer) && f.done();
}

inline bool read(Fields &f, Pan &x) {
    return f.type() == "PAN" && prefix(f, x.key) && f.number(x.key.tick) && f.number(x.demand) && f.done();
}

inline bool read(Fields &f, Maintenance &x) {
    return f.type() == "MAINTENANCE" && prefix(f, x.key) && f.number(x.generation) && f.number(x.key.tick) &&
        f.enumeration(x.operation) && f.number(x.pulse) && f.done();
}

inline bool read(Fields &f, Result &x) {
    return f.type() == "RESULT" && f.id(x.session) && f.number(x.seq) && x.seq != 0 &&
        f.enumeration(x.code) && f.number(x.generation) && f.done();
}

inline bool read(Fields &f, Status &x) {
    return f.type() == "STATUS" && f.id(x.session) && f.number(x.tick) && f.number(x.generation) &&
        f.enumeration(x.mode) && f.enumeration(x.reason) && f.number(x.cancel_generation) &&
        x.cancel_generation <= x.generation && f.enumeration(x.cancel_reason) && f.enumeration(x.rc) &&
        f.number(x.valid_axes) && x.valid_axes <= 3 && f.number(x.rc_drive) && f.number(x.rc_steer) &&
        f.enumeration(x.drive_choice) && f.enumeration(x.steer_choice) && f.enumeration(x.drive_source) &&
        f.enumeration(x.steer_source) && f.number(x.saturated) && x.saturated <= 3 &&
        f.number(x.accepted_drive) && f.number(x.selected_drive) && x.selected_drive <= x.accepted_drive &&
        f.number(x.drive_pwm) && f.number(x.steer_pwm) && f.number(x.pan_seq) && f.number(x.pan_pwm) && f.done();
}

struct Line {
    std::array<char, max_frame + 1> data{};
    std::size_t size{};
    Text view() const { return {data.data(), size}; }
};

class Writer {
public:
    template<class... T> Line frame(Text type, T... fields) {
        add(frame_marker);
        add(type);
        (add(fields), ...);
        if (line_.size > max_frame - 9 || failed_) return {};
        const auto crc = crc32(line_.view());
        std::snprintf(line_.data.data() + line_.size, 10, "%08lX\n", static_cast<unsigned long>(crc));
        line_.size += 9;
        return line_;
    }

private:
    void add(Text value) {
        if (value.size() >= max_frame - line_.size) {
            failed_ = true;
            return;
        }
        for (const char c : value) line_.data[line_.size++] = c;
        line_.data[line_.size++] = ',';
    }
    void add(const char *value) { add(Text{value}); }
    template<class T> void add(T value) {
        if constexpr (std::is_enum_v<T>) add(name(value));
        else {
            char text[24];
            if constexpr (std::is_signed_v<T>)
                std::snprintf(text, sizeof(text), "%lld", static_cast<long long>(value));
            else std::snprintf(text, sizeof(text), "%llu", static_cast<unsigned long long>(value));
            add(Text{text});
        }
    }
    Line line_;
    bool failed_{};
};

inline Line encode(Hello x) { return Writer{}.frame("HELLO", x.challenge); }
inline Line encode(Identity x) {
    return Writer{}.frame("ID", x.challenge, x.session, x.device, x.boot, x.role, x.board, x.build, x.profile);
}
inline Line encode(ModeRequest x) {
    return Writer{}.frame("MODE", x.key.session, x.key.seq, x.generation, x.key.tick, x.mode, x.drive, x.steer);
}
inline Line encode(Drive x) {
    return Writer{}.frame("DRIVE", x.key.session, x.key.seq, x.generation, x.key.tick, x.drive, x.steer);
}
inline Line encode(Pan x) { return Writer{}.frame("PAN", x.key.session, x.key.seq, x.key.tick, x.demand); }
inline Line encode(Maintenance x) {
    return Writer{}.frame("MAINTENANCE", x.key.session, x.key.seq, x.generation, x.key.tick, x.operation, x.pulse);
}
inline Line encode(Result x) { return Writer{}.frame("RESULT", x.session, x.seq, x.code, x.generation); }
inline Line encode(Status x) {
    return Writer{}.frame("STATUS", x.session, x.tick, x.generation, x.mode, x.reason, x.cancel_generation,
        x.cancel_reason, x.rc, x.valid_axes, x.rc_drive, x.rc_steer, x.drive_choice, x.steer_choice, x.drive_source,
        x.steer_source, x.saturated, x.accepted_drive, x.selected_drive, x.drive_pwm, x.steer_pwm, x.pan_seq, x.pan_pwm);
}

inline bool demand(int q) { return q >= -full && q <= full; }

} // namespace rc_car::wire
