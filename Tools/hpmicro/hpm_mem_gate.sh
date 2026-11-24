#!/usr/bin/env bash
# Post-flash memory acceptance gate for fmuv6x_hpm.
# Fail if free heap is too low or FC is not producing heartbeats.
#
# Usage: bash Tools/hpmicro/hpm_mem_gate.sh [port]
# Exit 0 = PASS, 1 = FAIL
set -euo pipefail

PORT="${1:-/dev/serial/by-id/usb-HPMicro_HPM_Ardupilot_2024051703-if00}"
# Must stay above HAL_HPM_HEAP_BOOT_MIN_BYTES (96KB) with margin after full init
MIN_FREE_BYTES="${HPM_MEM_GATE_MIN:-100000}"
WAIT_S="${HPM_MEM_GATE_WAIT:-25}"

python3 - "$PORT" "$MIN_FREE_BYTES" "$WAIT_S" <<'PY'
import sys, time
from pymavlink import mavutil

port, min_free, wait_s = sys.argv[1], int(sys.argv[2]), float(sys.argv[3])
print(f"gate: port={port} min_free={min_free} wait={wait_s}s")
m = mavutil.mavlink_connection(port, baud=115200, autoreconnect=True)
m.wait_heartbeat(timeout=15)
print("gate: heartbeat OK")

m.mav.command_long_send(
    m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    mavutil.mavlink.MAVLINK_MSG_ID_MEMINFO, 200000, 0, 0, 0, 0, 0)

free = None
mem_msgs = []
texts = []
t0 = time.time()
while time.time() - t0 < wait_s:
    msg = m.recv_match(blocking=True, timeout=1)
    if not msg:
        continue
    t = msg.get_type()
    if t == "MEMINFO":
        free = int(msg.freemem32)
        mem_msgs.append(free)
        print(f"gate: MEMINFO free={free}")
    elif t == "STATUSTEXT":
        s = msg.text
        if "MemGuard" in s or s.startswith("NET:") or "STACK" in s or "MALLOC" in s:
            print(f"gate: ST {s}")
            texts.append(s)

if free is None:
    print("gate: FAIL — no MEMINFO")
    sys.exit(1)
# Use the minimum observed free during the window (closer to steady-state)
steady = min(mem_msgs) if mem_msgs else free
print(f"gate: steady_min_free={steady} samples={len(mem_msgs)}")
if steady < min_free:
    print(f"gate: FAIL — free heap {steady} < {min_free}")
    sys.exit(1)

boot_ok = any("MemGuard: heap free=" in t for t in texts)
if not boot_ok:
    print("gate: WARN — no MemGuard boot banner yet (still PASS on heap)")
if any("CRIT" in t for t in texts):
    print("gate: FAIL — MemGuard CRITICAL seen")
    sys.exit(1)

print("gate: PASS")
sys.exit(0)
PY
