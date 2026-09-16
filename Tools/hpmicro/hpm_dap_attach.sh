#!/usr/bin/env bash
# Attach OpenOCD to HPM6750 via CMSIS-DAP (JTAG). Does not flash.
set -euo pipefail
AP="$(cd "$(dirname "$0")/../.." && pwd)"
WS="$(cd "$AP/.." && pwd)"
SDK="$AP/modules/hpm_sdk"
CFG="$SDK/boards/openocd"
OCD="${HPM_OPENOCD:-}"
if [[ -z "$OCD" ]]; then
  for cand in \
    "$WS/tools/hpm-openocd/usr/local/bin/openocd" \
    "$AP/tools/hpm-openocd/usr/local/bin/openocd"; do
    if [[ -x "$cand" ]]; then OCD="$cand"; break; fi
  done
fi
[[ -n "${OCD:-}" && -x "$OCD" ]] || OCD="$(command -v openocd || true)"
[[ -n "${OCD:-}" ]] || { echo "openocd not found; set HPM_OPENOCD"; exit 1; }

if ! lsusb -d 0d28:0204 >/dev/null 2>&1 && ! lsusb | grep -qi 'cmsis\|daplink\|mbed'; then
  echo "No CMSIS-DAP found. Map the probe first."
  exit 1
fi

echo "Using: $OCD"
exec "$OCD" -s "$CFG" \
  -c "adapter driver cmsis-dap; cmsis-dap backend usb_bulk; adapter speed 1000; transport select jtag; reset_config none" \
  -f "$CFG/soc/hpm6750-single-core.cfg" \
  -c "init; halt"
