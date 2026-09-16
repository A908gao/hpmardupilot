#!/usr/bin/env bash
# One-shot: install toolchain if needed, build fmuv6x_hpm, flash, verify MAVLink.
set -euo pipefail
AP="$(cd "$(dirname "$0")/../.." && pwd)"
WS="$(cd "$AP/.." && pwd)"
TC="${HPM_TOOLCHAIN:-$WS/tools/riscv-toolchain}"
URL='https://github.com/hpmicro/riscv-gnu-toolchain/releases/download/2023.10.18/rv32imac_zicsr_zifencei_multilib_b_ext-linux.tar.gz'

log() { printf '[hpm] %s\n' "$*"; }

if [[ ! -x "$TC/bin/riscv32-unknown-elf-gcc" ]]; then
  log "Installing RISC-V toolchain to $TC ..."
  mkdir -p "$TC"
  TGZ=/tmp/hpm-riscv-toolchain.tar.gz
  curl -L --fail --retry 3 -o "$TGZ" "$URL"
  if ! tar -xzf "$TGZ" -C "$TC" --strip-components=1 2>/dev/null; then
    rm -rf /tmp/hpm_tc_extract
    mkdir -p /tmp/hpm_tc_extract
    tar -xzf "$TGZ" -C /tmp/hpm_tc_extract
    GCC=$(find /tmp/hpm_tc_extract -type f -name riscv32-unknown-elf-gcc | head -1)
    PREFIX=$(dirname "$(dirname "$GCC")")
    cp -a "$PREFIX"/. "$TC"/
  fi
fi

export HPM_TOOLCHAIN="$TC"
# shellcheck disable=SC1091
source "$AP/Tools/hpmicro/hpm_env.sh"

python3 -c 'from pymavlink import mavutil' 2>/dev/null || pip3 install --user pymavlink

cd "$AP"
rm -f .lock-waf_linux_build
rm -f build/c4che/sitl_cache.py build/c4che/fmuv6x_hpm_cache.py build/c4che/_cache.py 2>/dev/null || true

log "Configuring fmuv6x_hpm..."
./waf configure --board fmuv6x_hpm
log "Building copter..."
./waf copter -j"$(nproc)"
BIN="$AP/build/fmuv6x_hpm/build/bin/arducopter/output/demo.bin"
test -f "$BIN"
log "Build OK: $BIN ($(stat -c%s "$BIN") bytes)"

if ! lsusb -d 0d28:0204 >/dev/null 2>&1; then
  log "Waiting for CMSIS-DAP (0d28:0204)..."
  for i in $(seq 1 30); do
    lsusb -d 0d28:0204 >/dev/null 2>&1 && break
    sleep 1
  done
fi
lsusb -d 0d28:0204 >/dev/null || { log "ERROR: no CMSIS-DAP"; exit 2; }

log "Flashing..."
pkill -9 openocd 2>/dev/null || true
sleep 1
bash "$AP/Tools/hpmicro/hpm_flash_fmuv6x.sh"

log "Waiting for USB CDC / MAVLink..."
sleep 5
python3 - <<'PY'
from pymavlink import mavutil
from collections import Counter
import time, os, sys

ports = [p for p in ("/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2") if os.path.exists(p)]
if not ports:
    print("FAIL: no /dev/ttyACM*")
    sys.exit(3)

m = None
for port in ports:
    try:
        mm = mavutil.mavlink_connection(port, baud=115200)
        t0 = time.time()
        while time.time() - t0 < 10:
            msg = mm.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg and getattr(msg, "autopilot", None) == 3:
                print(f"HB OK on {port} sys={mm.target_system} type={msg.type}")
                m = mm
                break
        if m:
            break
        mm.close()
    except Exception as e:
        print(port, e)

if not m:
    print("FAIL: no ArduPilot HEARTBEAT")
    sys.exit(4)

m.mav.request_data_stream_send(m.target_system, m.target_component, 0, 4, 1)
for mid, us in ((1, 200000), (29, 200000), (30, 100000)):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, us, 0, 0, 0, 0, 0)

c = Counter()
baro_ok = False
press = None
t0 = time.time()
while time.time() - t0 < 18:
    msg = m.recv_match(blocking=True, timeout=1)
    if not msg:
        continue
    t = msg.get_type()
    if t == "BAD_DATA":
        continue
    c[t] += 1
    if t == "SYS_STATUS" and c[t] <= 3:
        p, h = msg.onboard_control_sensors_present, msg.onboard_control_sensors_health
        baro_ok = bool(h & (1 << 3))
        print(f"SYS baro_h={baro_ok} present={hex(p)} health={hex(h)}")
    elif t == "SCALED_PRESSURE" and press is None:
        press = msg.press_abs
        print(f"PRESS {press:.1f} hPa temp={msg.temperature}")
    elif t == "ATTITUDE" and c[t] == 1:
        print(f"ATT roll={msg.roll:.3f} pitch={msg.pitch:.3f}")

print("counts", c.most_common(12))
if press is None and not baro_ok:
    print("WARN: no baro data yet (HB still OK)")
print("PASS: FC running (MAVLink ArduPilot heartbeat)")
sys.exit(0)
PY

log "All done."
