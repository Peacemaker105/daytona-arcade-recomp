// Telemetry packet builders (src/app/telemetry.h): the Forza data-out
// packet's byte layout against the dashboard reader's expectations, the JSON
// line's fields, and the DAYTONA_TELEMETRY spec parser.
#include "app/telemetry.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

namespace {
int failures = 0;
void check(bool ok, const char *what) {
    if (!ok) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

float f32(const uint8_t *p) {
    float v;
    std::memcpy(&v, p, 4);
    return v;
}
int32_t i32(const uint8_t *p) {
    int32_t v;
    std::memcpy(&v, p, 4);
    return v;
}
constexpr float G = 9.80665f;

app::TelemetrySample racing_sample() {
    app::TelemetrySample s;
    s.on = true;
    s.speed = 75.0f;
    s.rpm = 6400;
    s.steer = 0.5f;
    s.gas = 1.0f, s.brake = 0.0f;
    s.gear = 4;
    s.ffb = 0.25f, s.shake = 0.1f, s.effect = 4;
    s.long_g = -1.0f, s.lat_g = 1.5f, s.vert_g = 1.25f;
    s.yaw_rate = 0.8f;
    s.pos_x = 100.0f, s.pos_y = 5.0f, s.pos_z = -50.0f;
    s.have_pos = true;
    s.distance = 1234.5f;
    s.frame = 100;
    return s;
}

} // namespace

int main() {
    // The wire packet: 324 bytes, the reader's own channel conventions.
    uint8_t pkt[400];
    std::memset(pkt, 0xEE, sizeof pkt);
    const size_t n = app::telemetry_build_forza(racing_sample(), pkt);
    check(n == 324, "forza packet is 324 bytes");
    const uint8_t zero[400] = {};
    std::memset(pkt + 324, 0xEE, sizeof pkt - 324);
    check(i32(pkt + 0) == 1, "is_race_on 1 while racing");
    check(i32(pkt + 4) == uint32_t(100 * 1000 / 57.524), "timestamp ms from the frame");
    check(std::fabs(f32(pkt + 16) - 6400.0f) < 0.5f, "rpm");
    // The accelerations are m/s^2, X lateral, Y vertical without gravity, Z longitudinal
    check(std::fabs(f32(pkt + 20) - 1.5f * G) < 0.01f, "accel X is lateral g in m/s^2");
    check(std::fabs(f32(pkt + 24) - 0.25f * G) < 0.01f, "accel Y is vert_g minus 1 g in m/s^2");
    check(std::fabs(f32(pkt + 28) + 1.0f * G) < 0.01f, "accel Z is longitudinal g in m/s^2");
    check(f32(pkt + 32) == 0 && f32(pkt + 36) == 0, "velocity sideways and vertical stay 0");
    check(std::fabs(f32(pkt + 40) - 75.0f) < 0.01f, "velocity Z is the forward speed");
    check(f32(pkt + 44) == 0, "pitch rate 0");
    check(std::fabs(f32(pkt + 48) - 0.8f) < 0.01f, "yaw rate on Y");
    check(f32(pkt + 52) == 0, "roll rate 0");
    check(f32(pkt + 56) == 0 && f32(pkt + 60) == 0 && f32(pkt + 64) == 0, "yaw, pitch, roll stay 0");
    for (int i = 0; i < 4; i++) {
        const float travel = f32(pkt + 68 + 4 * i);
        check(travel > 0.02f && travel < 0.98f, "suspension within 0..1");
    }
    check(i32(pkt + 116) == 1 && i32(pkt + 128) == 1, "wheels on the ground");
    check(f32(pkt + 148) == 0.1f, "surface rumble carries the shake");
    check(i32(pkt + 212) == 1, "car ordinal");
    check(i32(pkt + 224) == 1, "drivetrain RWD");
    // The dash block at 244
    std::memset(pkt, 0xEE, sizeof pkt);
    app::telemetry_build_forza(racing_sample(), pkt);
    check(std::fabs(f32(pkt + 244 + 0) - 100.0f) < 0.01f, "dash position X");
    check(std::fabs(f32(pkt + 244 + 4) - 5.0f) < 0.01f, "dash position Y");
    check(std::fabs(f32(pkt + 244 + 8) + 50.0f) < 0.01f, "dash position Z");
    check(std::fabs(f32(pkt + 244 + 12) - 75.0f) < 0.01f, "dash speed m/s");
    check(std::fabs(f32(pkt + 244 + 48) - 1234.5f) < 0.1f, "dash distance");
    check(pkt[244 + 71] == 255, "throttle byte");
    check(pkt[244 + 72] == 0, "brake byte");
    check(pkt[244 + 74] == 0, "no handbrake");
    check(pkt[244 + 75] == 4, "gear");
    check(pkt[244 + 76] == int8_t(63), "steering byte + right");
    // A parked sample: everything at rest, is_race_on 0, no position
    app::TelemetrySample off;
    app::telemetry_build_forza(off, pkt);
    check(i32(pkt + 0) == 0, "is_race_on 0 off the race");
    check(f32(pkt + 24) == 0, "vertical at rest carries no gravity");
    check(f32(pkt + 244 + 0) == 0 && f32(pkt + 244 + 12) == 0, "parked: no position, no speed");
    check(pkt[244 + 75] == 0, "parked: gear 0");

    // The JSON line
    char buf[1024];
    const size_t j = app::telemetry_build_json(racing_sample(), buf, sizeof buf);
    check(j < sizeof buf, "json fits its buffer");
    check(buf[j - 1] == '\n', "json ends with a newline");
    check(std::strstr(buf, "\"on\":1") && std::strstr(buf, "\"speed\":75.00"), "json on and speed");
    check(std::strstr(buf, "\"gear\":4") && std::strstr(buf, "\"lat_g\":1.500"), "json gear and lat_g");
    check(std::strstr(buf, "\"pos_x\":100.00"), "json position");
    check(app::telemetry_build_json(off, buf, sizeof buf) < sizeof buf, "json off sample builds");

    // The spec parser
    std::string host;
    int port = 0;
    check(app::telemetry_parse_spec("127.0.0.1:20777", host, port) && host == "127.0.0.1" && port == 20777,
          "plain host:port parses, json default");
    check(app::telemetry_parse_spec("192.168.1.20:10050@forza", host, port) && host == "192.168.1.20" && port == 10050,
          "forza spec parses");
    check(app::telemetry_parse_spec("localhost:20777@both", host, port) && host == "localhost" && port == 20777,
          "both spec parses");
    check(!app::telemetry_parse_spec("no-port-here", host, port), "a spec without a port is refused");
    check(!app::telemetry_parse_spec("1.2.3.4:1@xml", host, port), "an unknown format is refused");

    if (failures) std::fprintf(stderr, "%d failures\n", failures);
    return failures;
}
