#!/usr/bin/env python3
"""Listen for the game's telemetry (docs/telemetry.md) on UDP 20777.

  telemetry_listen.py [seconds]   default 60

Prints one line per burst: packet counts, the Forza datagram sizes and the
JSON fields a rig reader would act on. For checking a motion platform while
the game runs with DAYTONA_TELEMETRY set.
"""
import json
import socket
import struct
import sys
import time

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("127.0.0.1", 20777))
s.settimeout(10)
deadline = time.time() + float(sys.argv[1] if len(sys.argv) > 1 else 60)
json_n = forza_n = 0
json_first = json_last = None
forza_sizes = set()
race_flags = set()
speeds = []
while time.time() < deadline:
    try:
        data, addr = s.recvfrom(4096)
    except socket.timeout:
        break
    if data.startswith(b'{"frame"'):
        json_n += 1
        try:
            d = json.loads(data)
        except ValueError:
            continue
        if json_first is None:
            json_first = d
        json_last = d
        race_flags.add(d.get("on"))
        speeds.append(d.get("speed", 0))
    else:
        forza_n += 1
        forza_sizes.add(len(data))
        if len(data) == 324:
            on = struct.unpack_from("<i", data, 0)[0]
            race_flags.add(bool(on))
print(f"json packets {json_n}, forza packets {forza_n}, forza sizes {sorted(forza_sizes)}")
print(f"is_race_on values seen {sorted(map(str, race_flags))}")
print(f"speed range {min(speeds) if speeds else 0:.1f}..{max(speeds) if speeds else 0:.1f} m/s")
if json_first:
    print("first:", json.dumps(json_first))
if json_last:
    print("last: ", json.dumps(json_last))
