// Telemetry for motion simulators and dashboards: one UDP datagram per game
// frame with the race state, read straight out of the game's RAM (nothing is
// hooked, nothing written), plus what the app itself knows: the wheel, pedals
// and shifter it feeds the game (Inputs), and the commands the game sends the
// drive board (DriveBoard models them, as force feedback does).
//
// Off unless DAYTONA_TELEMETRY is set (telemetry.h). Two formats:
// - json: one line per frame with every field named, for FlyPT Mover, a
//   script or anything else custom.
// - forza: the 324-byte Forza Horizon 4/5 data-out packet (232-byte sled +
//   12-byte gap + 80-byte dash block), what SimHub reads natively and most
//   motion software gets through it. The acceleration channels carry the
//   dashboard reader's own convention, m/s^2 rather than the Forza game's
//   g units: X lateral (+ right), Y vertical with gravity removed (+ up,
//   about 0 at rest), Z longitudinal (+ accelerating); the velocity block is
//   0, 0, forward speed; the angular block is yaw rate (+ right) on Y alone.
//
// The car's world position is the game's own physics, validated against the
// speed (displacement per frame tracks it) and against the course shape
// (the beginner oval's XZ path matches the track). Yaw rate and the g
// forces are derived from it. There is no pitch or roll anywhere in the
// player block: those stay 0, as does the heading the Forza packet would
// carry (the game keeps none that moves with the car; the camera's matrices
// are view-relative and have no world heading in them).
#include "app/telemetry.h"
#include "runtime/drive_board.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>

#ifdef _WIN32
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
using native_socket = SOCKET;
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>
using native_socket = int;
#endif

namespace app {

namespace {

constexpr double FRAME_HZ = 57.524; // the Model 2's vblank
constexpr float G = 9.80665f;

// The player block in work RAM, Revision A (MAME's daytona), found by the RAM
// probe (scripts/inputs + tools/m2run --ram-probe; docs/telemetry.md has the
// method). The speed read is the app's own (main.cpp's live meters). The
// physics block below is only confirmed on Revision A, so on daytona93 the
// position stays unmapped and the kinematics fall back to the speed alone.
constexpr uint32_t SPEED = 0x505198;       // raw integer; the HUD km/h is about 1.055x - 14.5
constexpr uint32_t POS_X = 0x50519c;       // metres, world frame: X and Z on the
constexpr uint32_t POS_Y = 0x5051a0;       // ground plane, Y up (0 on the road,
constexpr uint32_t POS_Z = 0x5051a4;       // hills to about 10)
constexpr uint32_t GEAR = 0x5052cc;        // 0 reverse/neutral, then 1-4
constexpr uint32_t RACE_FLAG = 0x505300;   // 1.0 while a race scene is loaded, 0 outside
constexpr uint32_t DISTANCE = 0x5078b4;    // metres driven this race, 16.16 fixed point

#ifdef M2_ROMSET
constexpr bool kPhysicsMapped = std::string_view(M2_ROMSET) == "daytona";
#else
constexpr bool kPhysicsMapped = false;
#endif

// The engine speed is not mapped: estimated from the speed inside each gear's
// band (m/s). Daytona tops out just under 300 km/h on the downhills.
constexpr float GEAR_TOP[4] = {17, 31, 47, 83.0f};
constexpr float RPM_IDLE = 800, RPM_MAX = 8000;

float rpm_of(float speed, int gear) {
    if (gear < 1 || gear > 4) return RPM_IDLE;
    const float low = gear >= 2 ? GEAR_TOP[gear - 2] : 0, top = GEAR_TOP[gear - 1];
    const float frac = std::clamp((speed - low) / (top - low), 0.0f, 1.0f);
    return RPM_IDLE + frac * (RPM_MAX - RPM_IDLE);
}

float speed_of(rt::M2Board &board) { // m/s, 0 while the raw integer is nonsense
    const int32_t raw = int32_t(board.read_dword(SPEED));
    if (raw <= 0 || raw >= 400) return 0;
    return std::max(0.0f, 1.055f * float(raw) - 14.5f) / 3.6f;
}

// ---- Kinematics kept between frames: velocity, yaw and the smoothed forces.
// Everything comes from the game's position, so a respawn or an attract-loop
// jump (tens of metres in a frame) is dropped rather than differentiated.
struct Kinematics {
    bool have_pos = false;
    float px = 0, py = 0, pz = 0;      // last accepted position
    float vx = 0, vz = 0;              // world velocity, m/s (smoothed)
    float course = 0;                  // unwrapped heading of the travel, rad
    float yaw_rate = 0;                // rad/s, + turning right (smoothed)
    float long_g = 0, lat_g = 0;       // smoothed, g
    float vert_g = 1;                  // 1 g at rest
    bool racing = false;               // the race scene flag's last state

    // One frame: the game's own values in, the derived forces out.
    void update(rt::M2Board &board, float speed, bool scene, float shake, float dt) {
        racing = scene;
        float accel_world_x = 0, accel_world_z = 0;
        if (scene) {
            const uint32_t wx = board.read_dword(POS_X), wy = board.read_dword(POS_Y), wz = board.read_dword(POS_Z);
            float fx, fy, fz;
            std::memcpy(&fx, &wx, 4), std::memcpy(&fy, &wy, 4), std::memcpy(&fz, &wz, 4);
            if (kPhysicsMapped && std::isfinite(fx) && std::isfinite(fy) && std::isfinite(fz)) {
                const float dx = fx - px, dy = fy - py, dz = fz - pz;
                const float step = std::sqrt(dx * dx + dy * dy + dz * dz);
                // The car covers at most about 1.5 m a frame; anything far
                // past that (and past what the speedometer says) is a
                // teleport, not motion.
                if (have_pos && step < std::max(4.0f, speed * dt * 3.0f)) {
                    const float nvx = dx / dt, nvz = dz / dt;
                    accel_world_x = (nvx - vx) / dt;
                    accel_world_z = (nvz - vz) / dt;
                    vx = 0.5f * (vx + nvx), vz = 0.5f * (vz + nvz);
                    if (step > 0.01f) {
                        // The travel heading: + is right (checked against the
                        // scripted steering phases), and the course on the
                        // screen turns with it.
                        float a = std::atan2(dx, dz);
                        float turn = a - course;
                        while (turn > 3.14159f) turn -= 6.28318f;
                        while (turn < -3.14159f) turn += 6.28318f;
                        course += turn;
                        yaw_rate += 0.4f * (turn / dt - yaw_rate);
                    }
                }
                px = fx, py = fy, pz = fz;
                have_pos = true;
            }
            // The world acceleration, in the car's frame: forward along the
            // travel, right across it. + lateral right, + longitudinal
            // accelerating, as the reader wants them.
            const float v = std::sqrt(vx * vx + vz * vz);
            if (v > 0.5f) {
                const float ux = vx / v, uz = vz / v; // the travel's unit vector
                const float along = accel_world_x * ux + accel_world_z * uz;
                const float across = accel_world_x * uz - accel_world_z * ux;
                long_g += 0.3f * (std::clamp(along / G, -2.5f, 2.5f) - long_g);
                lat_g += 0.35f * (std::clamp(across / G, -4.0f, 4.0f) - lat_g);
            } else if (speed < 1.0f) {
                long_g *= 0.8f, lat_g *= 0.8f;
            }
            vert_g = 1.0f + shake * 0.6f;
        } else {
            long_g *= 0.7f, lat_g *= 0.7f, yaw_rate *= 0.7f;
            vert_g += (1.0f - vert_g) * 0.3f;
            vx = vz = 0;
            have_pos = false; // the block freezes in menus: re-anchor on race entry
        }
    }
};

// ---- One frame's sample, and what the two formats are built from ----
// The drive board's state as of this frame: the frame's commands replayed.
void replay_drive_commands(rt::DriveBoard &drive, const uint8_t *commands, size_t n) {
    for (size_t i = 0; i < n; i++) drive.command(commands[i]);
}

TelemetrySample read_sample(rt::M2Board &board, const rt::Inputs &in, const rt::DriveBoard &drive,
                   const Kinematics &kin, uint64_t frame) {
    TelemetrySample s;
    s.frame = frame;
    s.on = kin.racing;
    s.speed = kin.racing ? speed_of(board) : 0;
    s.gear = 0;
    if (kPhysicsMapped && kin.racing) {
        const int32_t g = int32_t(board.read_dword(GEAR));
        if (g >= 0 && g <= 4) s.gear = g;
        const uint32_t raw = board.read_dword(DISTANCE);
        s.distance = float(raw >> 4) / 1048576.0f; // 16.16 fixed point, metres
    }
    int steer = int(in.steer) - 128;
    s.steer = std::clamp(float(steer) / 96.0f, -1.0f, 1.0f); // the app's own ADC scale
    s.gas = std::clamp((float(in.accel) - 32.0f) / 192.0f, 0.0f, 1.0f);
    s.brake = std::clamp((float(in.brake) - 32.0f) / 192.0f, 0.0f, 1.0f);
    s.rpm = rpm_of(s.speed, s.gear);
    s.ffb = drive.force();
    s.shake = drive.motor_on ? drive.level(drive.effect) * 0.4f : 0;
    s.effect = int(drive.effect);
    s.long_g = kin.long_g, s.lat_g = kin.lat_g, s.vert_g = kin.vert_g, s.yaw_rate = kin.yaw_rate;
    s.pos_x = kin.px, s.pos_y = kin.py, s.pos_z = kin.pz, s.have_pos = kin.have_pos && kin.racing;
    return s;
}

// ---- UDP ----
native_socket sock = native_socket(-1);
sockaddr_in dest{};
bool send_json = true, send_forza = false;


bool open_socket(const char *host, int port) {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa)) return false;
#endif
    sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == native_socket(-1)) return false;
    std::memset(&dest, 0, sizeof dest);
    dest.sin_family = AF_INET;
    dest.sin_port = htons(uint16_t(port));
#ifdef _WIN32
    inet_pton(AF_INET, host, &dest.sin_addr);
    if (dest.sin_addr.s_addr == INADDR_NONE) { // a name, then
        addrinfo *ai = nullptr;
        if (getaddrinfo(host, nullptr, nullptr, &ai) || !ai) return false;
        dest.sin_addr = ((sockaddr_in *)ai->ai_addr)->sin_addr;
        freeaddrinfo(ai);
    }
#else
    if (::inet_pton(AF_INET, host, &dest.sin_addr) != 1) { // a name, then
        addrinfo hints{}, *ai = nullptr;
        hints.ai_family = AF_INET;
        if (getaddrinfo(host, nullptr, &hints, &ai) || !ai) return false;
        dest.sin_addr = ((sockaddr_in *)ai->ai_addr)->sin_addr;
        freeaddrinfo(ai);
    }
#endif
    // A broadcast address (a rig on another machine's subnet, x.x.x.255) is
    // allowed only with the socket option set
    const uint32_t addr = ntohl(dest.sin_addr.s_addr);
    if (addr == 0xFFFFFFFF || (addr & 0xFF) == 0xFF) {
        int on = 1;
        setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (const char *)&on, sizeof on);
    }
    return true;
}

void send_frame(const char *buf, size_t n) {
#ifdef _WIN32
    sendto(sock, buf, int(n), 0, (sockaddr *)&dest, sizeof dest);
#else
    sendto(sock, buf, n, 0, (sockaddr *)&dest, sizeof dest);
#endif
}

void put_f32(uint8_t *p, float v) { std::memcpy(p, &v, 4); }
void put_i32(uint8_t *p, int32_t v) { std::memcpy(p, &v, 4); }
void put_u32(uint8_t *p, uint32_t v) { std::memcpy(p, &v, 4); }

// The 324-byte Forza Horizon data-out packet: 232-byte sled, 12-byte gap, the
// 80-byte dash block. SimHub picks the format by the datagram's size. The
// acceleration channels carry the dashboard reader's convention, m/s^2, not
// the Forza game's g units (see the file comment).


} // namespace

bool telemetry_parse_spec(const char *e, std::string &host, int &port) {
    std::string spec(e);
    const size_t colon = spec.find(':'), at = spec.find('@', colon == std::string::npos ? 0 : colon);
    std::string fmt = at == std::string::npos ? "json" : spec.substr(at + 1);
    if (at != std::string::npos) spec.resize(at);
    char hostbuf[128];
    if (std::sscanf(spec.c_str(), "%127[^:]:%d", hostbuf, &port) != 2) {
        std::printf("[telemetry] DAYTONA_TELEMETRY is \"%s\": want host:port[@json|@forza|@both]\n", e);
        return false;
    }
    host = hostbuf;
    send_json = fmt == "json" || fmt == "both";
    send_forza = fmt == "forza" || fmt == "both";
    if (!send_json && !send_forza) {
        std::printf("[telemetry] unknown format \"%s\": json, forza or both\n", fmt.c_str());
        return false;
    }
    return true;
}

size_t telemetry_build_forza(const TelemetrySample &s, uint8_t *p) {
    std::memset(p, 0, 324);
    put_i32(p + 0, s.on ? 1 : 0);
    put_u32(p + 4, uint32_t(s.frame * 1000 / 57.524) & 0xFFFFFFFF); // timestamp ms, wraps
    put_f32(p + 8, RPM_MAX), put_f32(p + 12, RPM_IDLE), put_f32(p + 16, s.rpm);
    put_f32(p + 20, s.lat_g * G);            // X: lateral, + right, m/s^2
    put_f32(p + 24, (s.vert_g - 1) * G);     // Y: vertical, gravity gone, + up, m/s^2
    put_f32(p + 28, s.long_g * G);           // Z: longitudinal, + accelerating, m/s^2
    put_f32(p + 32, 0), put_f32(p + 36, 0);  // velocity: sideways, vertical
    put_f32(p + 40, s.speed);                // forward, m/s
    put_f32(p + 44, 0), put_f32(p + 48, s.yaw_rate), put_f32(p + 52, 0); // angular: yaw rate on Y
    put_f32(p + 56, 0), put_f32(p + 60, 0), put_f32(p + 64, 0); // yaw, pitch, roll: none in RAM
    // Suspension, a plausible dressing of the forces (the game's real
    // suspension is not mapped): 0.5 at rest, braking on the front, cornering
    // on the outer (left, turning right) wheels
    const float fl = std::clamp(0.5f + 0.10f * s.lat_g - 0.12f * s.long_g, 0.02f, 0.98f);
    const float fr = std::clamp(0.5f - 0.10f * s.lat_g - 0.12f * s.long_g, 0.02f, 0.98f);
    const float rl = std::clamp(0.5f + 0.10f * s.lat_g + 0.12f * s.long_g, 0.02f, 0.98f);
    const float rr = std::clamp(0.5f - 0.10f * s.lat_g + 0.12f * s.long_g, 0.02f, 0.98f);
    put_f32(p + 68, fl), put_f32(p + 72, fr), put_f32(p + 76, rl), put_f32(p + 80, rr);
    // Slip at the driven (rear) wheels: spinning away, locking brakes
    float slip = 0;
    if (s.gas > 0.7f && s.speed < 14.0f) slip = s.gas * (1 - s.speed / 14.0f) * 0.8f;
    if (s.brake > 0.7f && s.speed > 5.0f && s.long_g < -0.5f) slip = std::max(slip, 0.3f * s.brake);
    put_f32(p + 84, 0), put_f32(p + 88, 0), put_f32(p + 92, slip), put_f32(p + 96, slip);
    const float wheel_rpm = s.speed * 30.8f; // a wheel of about 1.95 m a turn
    put_f32(p + 100, wheel_rpm), put_f32(p + 104, wheel_rpm), put_f32(p + 108, wheel_rpm), put_f32(p + 112, wheel_rpm);
    for (int i = 0; i < 4; i++) put_i32(p + 116 + 4 * uint32_t(i), 1); // wheels on the ground
    const float temp = 90 + s.speed * 0.3f + slip * 30.0f; // Fahrenheit on the wire
    for (int i = 0; i < 4; i++) put_f32(p + 132 + 4 * uint32_t(i), temp);
    for (int i = 0; i < 4; i++) put_f32(p + 148 + 4 * uint32_t(i), s.shake); // surface rumble
    for (int i = 0; i < 4; i++) put_f32(p + 164 + 4 * uint32_t(i), 0);      // slip angle
    for (int i = 0; i < 4; i++) put_f32(p + 180 + 4 * uint32_t(i), slip);   // combined slip
    const float travel = std::clamp(0.5f * (fl + fr + rl + rr), 0.0f, 1.0f);
    for (int i = 0; i < 4; i++) put_f32(p + 196 + 4 * uint32_t(i), travel);
    put_i32(p + 212, 1);                        // car ordinal
    put_i32(p + 216, 1), put_i32(p + 220, 999); // class, performance index
    put_i32(p + 224, 1);                        // drivetrain: RWD
    put_i32(p + 228, 8);                        // cylinders (the stock car's V8)
    // The dash block, after the 12-byte gap at 232
    uint8_t *d = p + 244;
    if (s.have_pos) put_f32(d + 0, s.pos_x), put_f32(d + 4, s.pos_y), put_f32(d + 8, s.pos_z); // the game's own world position
    put_f32(d + 12, s.speed); // m/s
    put_f32(d + 16, 300.0f * s.rpm * 6.28318f / 60.0f * s.gas); // power W, a flat 300 Nm with the throttle
    put_f32(d + 20, 300.0f * s.gas); // torque Nm
    for (int i = 0; i < 4; i++) put_f32(d + 24 + 4 * uint32_t(i), temp);
    put_f32(d + 40, 0);    // boost
    put_f32(d + 44, 0.5f); // fuel
    put_f32(d + 48, s.distance); // metres driven this race (the game's own)
    put_f32(d + 52, 0), put_f32(d + 56, 0), put_f32(d + 60, 0); // best, last, current lap: not mapped
    put_f32(d + 64, float(s.frame) / float(FRAME_HZ)); // time since the app started
    const uint16_t lap_no = 0;
    std::memcpy(d + 68, &lap_no, 2);
    d[70] = 0; // race position: not mapped
    d[71] = uint8_t(s.gas * 255), d[72] = uint8_t(s.brake * 255);
    d[73] = 0; // clutch
    d[74] = 0; // handbrake (Daytona has none)
    d[75] = uint8_t(std::clamp(s.gear, 0, 4));
    d[76] = int8_t(std::clamp(s.steer, -1.0f, 1.0f) * 127.0f);
    d[77] = 0x80, d[78] = 0x80; // driving line, AI brake: none (read back as -1)
    d[79] = 0;                  // the packet's last byte: padding
    return 324;
}

size_t telemetry_build_json(const TelemetrySample &s, char *out, size_t cap) {
    std::snprintf(out, cap,
                  "{\"frame\":%llu,\"on\":%d,\"speed\":%.2f,\"speed_kmh\":%.1f,\"rpm\":%.0f,\"gear\":%d,"
                  "\"steer\":%.3f,\"gas\":%.3f,\"brake\":%.3f,"
                  "\"ffb_force\":%.3f,\"shake\":%.3f,\"ffb_effect\":%d,"
                  "\"long_g\":%.3f,\"lat_g\":%.3f,\"vert_g\":%.3f,\"yaw_rate\":%.3f,"
                  "\"pos_x\":%.2f,\"pos_y\":%.2f,\"pos_z\":%.2f,\"distance\":%.1f}\n",
                  (unsigned long long)s.frame, s.on ? 1 : 0, s.speed, s.speed * 3.6f, s.rpm, s.gear, s.steer, s.gas,
                  s.brake, s.ffb, s.shake, s.effect, s.long_g, s.lat_g, s.vert_g, s.yaw_rate, s.pos_x, s.pos_y,
                  s.pos_z, s.distance);
    return std::strlen(out);
}

void telemetry_frame(rt::M2Board &board, const rt::Inputs &inputs, const uint8_t *commands, size_t commands_n,
                     uint64_t frame) {
    // The drive board's state as of this frame, then the app's settings once.
    static rt::DriveBoard drive;
    replay_drive_commands(drive, commands, commands_n);
    static bool tried = false;
    if (!tried) {
        tried = true;
        const char *e = std::getenv("DAYTONA_TELEMETRY");
        if (!e || !*e) return;
        std::string host;
        int port = 0;
        if (!telemetry_parse_spec(e, host, port)) return;
        if (!open_socket(host.c_str(), port)) {
            std::printf("[telemetry] could not open a UDP socket for %s:%d\n", host.c_str(), port);
            return;
        }
        std::printf("[telemetry] sending %s to %s:%d at %.1f Hz\n",
                    send_json && send_forza ? "json+forza" : send_forza ? "forza" : "json", host.c_str(), port, FRAME_HZ);
    }
    if (sock == native_socket(-1)) return;

    static Kinematics kin;
    const bool racing = [&board] {
        if (!kPhysicsMapped) return false;
        const uint32_t raw = board.read_dword(RACE_FLAG);
        float f;
        std::memcpy(&f, &raw, 4);
        return std::isfinite(f) && f > 0.5f;
    }();
    const float speed = racing ? speed_of(board) : 0;
    const float shake = drive.motor_on ? drive.level(drive.effect) * 0.4f : 0;
    kin.update(board, speed, racing, shake, float(1.0 / FRAME_HZ));
    const TelemetrySample s = read_sample(board, inputs, drive, kin, frame);
    char buf[1024];
    if (send_json) {
        const size_t n = telemetry_build_json(s, buf, sizeof buf);
        send_frame(buf, n);
    }
    if (send_forza) {
        uint8_t pkt[324];
        telemetry_build_forza(s, pkt);
        send_frame((const char *)pkt, sizeof pkt);
    }
}

} // namespace app
