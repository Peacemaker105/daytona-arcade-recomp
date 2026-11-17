// Telemetry for motion simulators and dashboards (telemetry.cpp).
//
// Off by default. Set DAYTONA_TELEMETRY to turn it on:
//   DAYTONA_TELEMETRY=127.0.0.1:20777             JSON lines, one per frame
//   DAYTONA_TELEMETRY=127.0.0.1:20777@forza       Forza Horizon data-out packet
//                                                 (324 bytes, what SimHub reads)
//   DAYTONA_TELEMETRY=127.0.0.1:20777@both        both, JSON first
// One UDP datagram per game frame (57.52 Hz) to host:port, sent from the app's
// frame loop after the game's frame has run. Nothing is hooked and nothing is
// written to the game's RAM: the car's state is read straight out of it. The
// packets keep flowing in the menus and attract mode with is_race_on 0 and the
// kinematics at rest; when the game pauses or quits they stop, which the
// reader treats as "home the rig".
#pragma once

#include "runtime/game_loop.h"

#include <cstddef>
#include <cstdint>
#include <string>

namespace app {

// One frame's sample. The kinematics (long_g, lat_g, vert_g, yaw_rate) are
// derived from the game's own world position by the caller-side state; the
// rest is read or handed in as it is. The sample and the two packet builders
// are exported so tests can check the wire bytes without a board.
struct TelemetrySample {
    bool on = false;       // racing: what Forza's is_race_on and SimHub's active mean
    float speed = 0;       // m/s (the game's own)
    float rpm = 0;         // estimate
    float steer = 0;       // -1 left .. +1 right, what the app feeds the game
    float gas = 0, brake = 0;
    int gear = 0;          // the game's own, 0 reverse/neutral .. 4
    float ffb = 0, shake = 0; // the drive board's constant force, and the rough shake level
    int effect = 0;        // the drive board's effect, as DriveBoard::Effect's number
    float long_g = 0, lat_g = 0, vert_g = 1;
    float yaw_rate = 0;
    float pitch = 0, roll = 0; // rad, the game's own: nose-down positive, lean-left positive
    float pos_x = 0, pos_y = 0, pos_z = 0;
    bool have_pos = false;
    float distance = 0;    // metres driven this race
    float susp_l = 0, susp_r = 0; // the game's front suspension pair, as read
    uint64_t frame = 0;
};

// The 324-byte Forza Horizon 4/5 data-out packet: 232-byte sled, 12 zero
// bytes, 80-byte dash block. The acceleration channels carry the dashboard
// reader's convention, m/s^2 rather than the Forza game's g units: X lateral
// (+ right), Y vertical with gravity removed (+ up), Z longitudinal
// (+ accelerating). Always writes the whole packet, returns 324.
size_t telemetry_build_forza(const TelemetrySample &s, uint8_t *out);
// One JSON line with every field named, newline included.
size_t telemetry_build_json(const TelemetrySample &s, char *out, size_t cap);
// "host:port[@json|@forza|@both]"; says and returns false when it is not one.
bool telemetry_parse_spec(const char *e, std::string &host, int &port);

// board: the game's memory, read only. inputs: what the game saw this frame.
// commands: the drive-board bytes this frame's run appended (the caller's own
// drain of them is untouched).
void telemetry_frame(rt::M2Board &board, const rt::Inputs &inputs, const uint8_t *commands, size_t commands_n,
                     uint64_t frame);

} // namespace app
