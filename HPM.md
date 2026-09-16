# HPM ArduPilot（fmuv6x_hpm）

本仓库是面向 **HPMicro HPM6750 / FMU V6X** 的 ArduPilot 精简树：

- HAL：`AP_HAL_HPMICRO`（FreeRTOS + hpm_sdk）
- 仿真：保留 SITL
- 已去掉 ChibiOS / ESP32 / Linux SBC / QURT HAL 与 `modules/ChibiOS`
- 目标板：`fmuv6x_hpm`（ICM45686、ICP20100、SPI0 FRAM、USB MAVLink）

## 编译

需要 HPMicro `riscv32-unknown-elf` GCC（13.2，[2023.10.18 发布包](https://github.com/hpmicro/riscv-gnu-toolchain/releases/tag/2023.10.18)）。烧录需要带 `hpm_xpi` 的 OpenOCD。

```bash
git clone --recursive git@github.com:A908gao/hpmardupilot.git
cd hpmardupilot
# 一键：装工具链、编译、烧录、检测心跳
./Tools/hpmicro/hpm_setup_build_flash.sh
```

或分步：

```bash
source Tools/hpmicro/hpm_env.sh
./waf configure --board fmuv6x_hpm
./waf copter -j$(nproc)
./Tools/hpmicro/hpm_flash_fmuv6x.sh
```

SITL：

```bash
./waf configure --board sitl
./waf copter -j$(nproc)
```

不要把 `sitl` 与 `fmuv6x_hpm` 的 `build/c4che` 混用；切换板型时删掉另一侧的 `*_cache.py`。

## 不进仓库的内容

工具链、OpenOCD 二进制、Docker 镜像、`build/` 产物不提交。请本地安装或放在仓库上一级的 `tools/` 目录。
