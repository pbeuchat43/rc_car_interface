#include "rc_car/serial.hpp"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <pty.h>
#include <unistd.h>

using namespace rc_car;
using namespace rc_car::wire;
#define CHECK(x) do { if (!(x)) { std::cerr << __LINE__ << ": " #x "\n"; std::abort(); } } while (0)

namespace {
constexpr Text device = "1111111111111111", challenge = "0123456789ABCDEF", session_token = "2222222222222222";
const Identity identity{challenge, session_token, device, "3333333333333333", "RCCAR",
    "waveshare_rp2040_zero", "build", "profile"};

std::string fixture(const std::string &dir, Text name) {
    std::ifstream file(dir + "/" + std::string(name) + ".wire", std::ios::binary);
    CHECK(file);
    return {std::istreambuf_iterator<char>(file), {}};
}

void push(Session &session, Line line, uint64_t now) {
    for (char c : line.view()) session.push(c, now);
}

void session_behavior() {
    Session s({std::string(device), "profile", "build"}, std::string(challenge), 0);
    CHECK(!s.ready(1) && s.hello(1).size && !s.hello(5).size && s.hello(101).size);
    Status status{};
    status.session = session_token;
    status.tick = 1234;
    status.generation = 9;
    status.mode = Mode::automatic;
    push(s, encode(status), 101);
    CHECK(!s.ready(101));
    push(s, encode(identity), 102);
    push(s, encode(status), 103);
    CHECK(s.ready(103) && s.token() == session_token);
    CHECK(!s.drive(104, 8, 100, 100).size);
    const auto d = s.drive(104, 9, 100, 100);
    CHECK(d.size);
    Fields df(d.view().substr(0, d.size - 1));
    Drive drive;
    CHECK(read(df, drive) && drive.key.tick == 1234 && drive.key.seq == 1);
    const auto m = s.mode(104, 9, Mode::automatic, Choice::rc, Choice::rc);
    Fields mf(m.view().substr(0, m.size - 1));
    ModeRequest mode;
    CHECK(read(mf, mode) && mode.mode == Mode::manual && mode.drive == Choice::none);
    push(s, encode(Result{session_token, 1, Code::ok, 9}), 105);
    CHECK(!s.take_result());
    push(s, encode(Result{session_token, 2, Code::ok, 10}), 106);
    CHECK(s.take_result()->generation == 10 && !s.take_result());
    push(s, encode(status), 200);
    CHECK(s.ready(252) && !s.ready(253));
    CHECK(!s.pan(253, 10).size);
    status.tick++;
    status.generation++;
    status.session = "4444444444444444";
    push(s, encode(status), 254);
    CHECK(!s.ready(254));
    CHECK(deadline(1'100'000'000, 1'200'000'000, 500) == 600);
    CHECK(!deadline(1'000'000'000, 1'200'000'000, 500));
    CHECK(!deadline(1'221'000'000, 1'200'000'000, 500));
    CHECK(quantize(-0.25005) == -2501 && !quantize(1.1));
}

void pty_round_trip(const std::string &fixture_dir) {
    int controller, endpoint;
    char path[128];
    CHECK(openpty(&controller, &endpoint, path, nullptr, nullptr) == 0);
    Port port(path);
    ::close(endpoint);
    bool locked = false;
    try { Port conflicting(path); } catch (const std::system_error &) { locked = true; }
    CHECK(locked);
    Session session({std::string(device), "profile", ""}, std::string(challenge), 0);
    port.write(session.hello(1));
    char bytes[256];
    const auto count = ::read(controller, bytes, sizeof(bytes));
    CHECK(count > 0 && Text(bytes, count) == fixture(fixture_dir, "hello"));
    const auto id = encode(identity);
    CHECK(::write(controller, id.data.data(), 12) == 12);
    port.read(session, 2);
    CHECK(session.token().empty());
    CHECK(::write(controller, id.data.data() + 12, id.size - 12) == ssize_t(id.size - 12));
    port.read(session, 3);
    CHECK(session.token() == session_token);
    ::close(controller);
}

void discovery() {
    const std::vector<UsbDevice> devices{{"/dev/ttyACM1", "1010101010101010"}, {"/dev/ttyACM2", std::string(device)}};
    CHECK(select_usb(devices, std::string(device)).path == "/dev/ttyACM2");
    bool ambiguous = false;
    try { select_usb(devices, ""); } catch (const std::runtime_error &) { ambiguous = true; }
    CHECK(ambiguous);
}
}

int main(int argc, char **argv) {
    CHECK(argc == 2);
    const auto golden = fixture(argv[1], "drive");
    CHECK(encode(Drive{{challenge, 2, 1234}, 7, -2500, 500}).view() == golden);
    discovery();
    session_behavior();
    pty_round_trip(argv[1]);
    std::cout << "RC1 host codec, freshness, discovery and PTY checks passed\n";
}
