#!/usr/bin/env bash
# Configure native HPM build environment (no Docker).
# Usage from ardupilot tree: source Tools/hpmicro/hpm_env.sh
set -euo pipefail
AP="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# Workspace layout: HPM_Ardupilot/tools next to ardupilot/
WS="$(cd "$AP/.." && pwd)"
TC="${HPM_TOOLCHAIN:-}"
if [[ -z "$TC" ]]; then
  for cand in "$WS/tools/riscv-toolchain" "$AP/tools/riscv-toolchain" "$HOME/riscv-toolchain"; do
    if [[ -x "$cand/bin/riscv32-unknown-elf-gcc" ]]; then
      TC="$cand"
      break
    fi
  done
fi
SDK="${HPM_SDK_BASE:-$AP/modules/hpm_sdk}"

if [[ -z "${TC:-}" || ! -x "$TC/bin/riscv32-unknown-elf-gcc" ]]; then
  echo "Missing riscv32-unknown-elf-gcc. Set HPM_TOOLCHAIN or run Tools/hpmicro/hpm_setup_build_flash.sh" >&2
  return 1 2>/dev/null || exit 1
fi

export PATH="$TC/bin:$PATH"
export HPM_SDK_BASE="$SDK"
export GNURISCV_TOOLCHAIN_PATH="$TC"

echo "HPM env ready: $(riscv32-unknown-elf-gcc --version | head -1)"
echo "  HPM_SDK_BASE=$HPM_SDK_BASE"
echo "  GNURISCV_TOOLCHAIN_PATH=$GNURISCV_TOOLCHAIN_PATH"
