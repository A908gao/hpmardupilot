#!/usr/bin/env bash
# Flash fmuv6x_hpm arducopter via CMSIS-DAP (JTAG, usb_bulk).
set -euo pipefail
AP="$(cd "$(dirname "$0")/../.." && pwd)"
WS="$(cd "$AP/.." && pwd)"
SDK="$AP/modules/hpm_sdk"
CFG="$SDK/boards/openocd"
ELF="$AP/build/fmuv6x_hpm/build/bin/arducopter/output/demo.elf"
OCD="${HPM_OPENOCD:-}"
if [[ -z "$OCD" ]]; then
  for cand in \
    "$WS/tools/hpm-openocd/usr/local/bin/openocd" \
    "$AP/tools/hpm-openocd/usr/local/bin/openocd"; do
    if [[ -x "$cand" ]]; then OCD="$cand"; break; fi
  done
fi
[[ -n "${OCD:-}" && -x "$OCD" ]] || { echo "openocd not found; set HPM_OPENOCD"; exit 1; }
[[ -f "$ELF" ]] || { echo "ELF missing: $ELF"; exit 1; }
if ! lsusb -d 0d28:0204 >/dev/null 2>&1; then
  echo "No CMSIS-DAP (0d28:0204). Map DAPLink into the VM first."
  exit 1
fi

SCRIPTS="$(cd "$(dirname "$OCD")/../share/openocd/scripts" && pwd)"
echo "Using: $OCD"
echo "ELF:   $ELF"
# V6X NOR uses XPI0 2nd pin group (board.c option[2]=0x1000)
exec "$OCD" -s "$CFG" -s "$SCRIPTS" \
  -c "adapter driver cmsis-dap; cmsis-dap backend usb_bulk; adapter speed 1000; transport select jtag; reset_config srst_only; adapter srst delay 500" \
  -f "$CFG/soc/hpm6750-single-core.cfg" \
  -c "
flash bank xpi0 hpm_xpi 0x80000000 0x400000 1 1 \$_TARGET0 0xF3040000 0x7 0x1000
\$_TARGET0 configure -event reset-init {
  mww 0xF4000800 0xFFFFFFFF
  mww 0xF4000810 0xFFFFFFFF
  mww 0xF4000820 0xFFFFFFFF
  mww 0xF4000830 0xFFFFFFFF
}
init
reset halt
program $ELF verify
reset run
shutdown
"
