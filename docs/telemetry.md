# Motion-simulator telemetry

The app can broadcast the car's state over UDP while you race, one datagram
per game frame (57.52 Hz), for a motion platform or a dashboard. It is off by
default; set `DAYTONA_TELEMETRY` to turn it on:

    DAYTONA_TELEMETRY=127.0.0.1:20777             JSON lines, one per frame
    DAYTONA_TELEMETRY=127.0.0.1:20777@forza       Forza Horizon data-out packet
                                                  (324 bytes, what SimHub reads)
    DAYTONA_TELEMETRY=127.0.0.1:20777@both        both, JSON first

Then start the game as usual. Nothing else to configure: the game reads the
car's state out of its own RAM each frame and sends it; nothing is hooked and
nothing is written to the game's memory, so with telemetry off nothing changes
at all. `scripts/telemetry_listen.py` listens on the same port and prints what
arrives, for checking a rig:

    python scripts/telemetry_listen.py [seconds]

## What each format carries

**forza** is the 324-byte Forza Horizon 4/5 data-out packet (232-byte sled +
12 zero bytes + 80-byte dash block, little-endian). SimHub and most motion
software dispatch on the datagram size, so the byte layout is fixed. The
acceleration channels carry the dashboard reader's convention, m/s^2 rather
than the Forza game's g units — this is the part to keep identical if the
packet ever changes:

| Offset | Field | Sent |
|---|---|---|
| 0 | is_race_on (i32) | 1 only while a race scene is loaded |
| 4 | timestamp_ms | frame × 1000 ÷ 57.524, wraps |
| 8/12/16 | max/idle/current RPM | RPM is estimated from the speed inside each gear's band (the game maps no tacho) |
| 20 | AccelerationX | lateral, + right, m/s² |
| 24 | AccelerationY | vertical with gravity removed, + up, m/s², about 0 at rest |
| 28 | AccelerationZ | longitudinal, + accelerating, m/s² |
| 32/36 | VelocityX/Y | 0 |
| 40 | VelocityZ | forward speed, m/s |
| 44/48/52 | AngularVelocity | pitch rate 0, yaw rate on Y (+ turning right, rad/s), roll rate 0 |
| 56/60/64 | yaw/pitch/roll | 0: the game maps no orientation angles (see below) |
| 68–84 | suspension ×4 | FL FR RL RR, 0..1, dressed from the g forces (0.5 at rest) |
| 84–99 | tyre slip ×4 | estimated from throttle/brake and speed at the rear |
| 100–115 | wheel RPM ×4 | speed ÷ wheel circumference |
| 116–131 | wheels on ground (i32) | 1 |
| 132–147 | tyre temp ×4 | dressed from speed and slip, °F |
| 148–163 | surface rumble ×4 | the drive board's effect level (bumps and hits) |
| 196–211 | suspension travel ×4 | as the suspension block |
| 212–231 | car/class/performance/drivetrain/cylinders | stock car ordinals |
| 244–255 | world position xyz | the game's own, metres (0 off the race) |
| 256 | speed m/s | as offset 40 |
| 292 | distance | metres driven this race, the game's own counter |
| 308 | race time | frames since the app started ÷ 57.524 |
| 315/316 | throttle/brake bytes | 0..255 |
| 318/319 | handbrake/gear | handbrake 0 (Daytona has none); gear 0–4, the game's own |
| 320 | steering (i8) | −1 left .. +1 right, ×127 |

**json** is one line per frame with every field named, for FlyPT Mover, a
script or anything custom:

    {"frame":1234,"on":1,"speed":58.4,"speed_kmh":210.2,"rpm":5200,"gear":3,
     "steer":-0.333,"gas":1.000,"brake":0.000,"ffb_force":-0.220,"shake":0.000,
     "ffb_effect":3,"long_g":0.412,"lat_g":-1.050,"vert_g":1.000,"yaw_rate":0.487,
     "pos_x":101.02,"pos_y":2.91,"pos_z":-188.49,"distance":4031.7}

## Behaviour the reader can rely on

- Packets **keep flowing** in the menus and attract mode with `is_race_on` 0,
  speed 0, gear 0 and the forces at rest — the rig stays idle but the stream
  stays alive. In attract the game's demo car moves, and it is deliberately
  not reported: its speed cell is shared with the player's, but the race flag
  gates the whole packet, so a demo lap never moves the platform.
- When the game is paused or closed the frames stop and the stream goes
  silent — there is no "stopped" packet. A reader should watchdog about
  300 ms of silence and home the rig, which is what the dashboards already
  do.
- One datagram per format per frame, unicast, no handshake. Broadcast to a
  subnet (`x.x.x.255`) works, the socket option is set as needed.

## Where the data comes from

All of it is read from the game's work RAM each frame; these are Revision A
(`daytona`, MAME's set) addresses, found with a scripted headless race and
`tools/m2run --ram-probe` (analysis of our own; rule 7):

| Address | Meaning |
|---|---|
| 0x505198 | speed, raw integer; the HUD km/h is about 1.055 × x − 14.5 (already known: the live meters read it) |
| 0x50519c/a0/a4 | the car's world position, X/Y/Z, metres: X and Z on the ground plane, Y up (0 on the road, hills to about 10) |
| 0x5052cc | gear, 0 reverse/neutral then 1–4 |
| 0x505300 | 1.0 float while a race scene is loaded, 0 outside it |
| 0x5078b4 | metres driven this race, 16.16 fixed point |

The position is the strongest check in the chain and the source of everything
derived: plotted over a race it draws the beginner oval and its infield loop
exactly, and its displacement per frame tracks the speedometer (0.90–1.0 ×
speed × frame time — the residual is the HUD formula's own approximation).
Yaw rate is the travel heading's rate of change (+ right, checked against the
scripted steering phases), and the accelerations come from differentiating the
velocity in the car's frame: longitudinal along it, lateral across it. A
respawn or the attract loop's reset jumps the position tens of metres in one
frame; moves past what the speedometer allows are dropped rather than
differentiated, so a crash never kicks the platform.

The matrices that would carry pitch, roll and a true heading are built for
the camera: the display list's own matrix and translation writes are all
view-relative, and no winding angle turns with the laps anywhere in the
probed RAM (work RAM 0x500000–0x600000, main RAM 0x200000–0x21ffff and buffer
RAM 0x900000–0x91ffff were each dumped every few frames over a full scripted
race). Pitch and roll therefore stay 0, and yaw rate is reported directly
rather than a heading.

`daytona93` (Deluxe '93) has not had its physics block probed: on that set the
emitter sends the inputs, the drive board and the speed, with the position
fields 0, until the same probe is run against `roms/daytona93.zip`.

## Testing it

`test_app_telemetry` checks the wire bytes: the acceleration channels'
convention, the dash block, the parked sample (no gravity on the vertical
channel, gear 0, no position) and the spec parser. The end-to-end run is the
game itself with `DAYTONA_TELEMETRY=127.0.0.1:20777@both` and the listener
script: one JSON and one 324-byte Forza datagram per frame, `on` 0 until a
race starts, the position tracing the course as you drive.
