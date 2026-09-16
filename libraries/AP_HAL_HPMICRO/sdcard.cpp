/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Filesystem/AP_Filesystem.h>

#include "sdcard.h"

#include "hpm_sdmmc_sd.h"
#include "hpm_gpio_drv.h"
#include "board.h"
#include "pinmux.h"
#include "ff.h"
#include "diskio.h"

#ifdef HAL_HPMICRO_SDCARD

FATFS s_sd_disk;
static bool sdcard_running;
static HAL_Semaphore sem;
static const TCHAR sd_root[] = { DEV_SD + '0', ':', '/', '\0' };

static const AP_HAL::HAL& hpmhal = AP_HAL::get_HAL();

const char *show_error_string(FRESULT fresult)
{
    switch (fresult) {
    case FR_OK: return "succeeded";
    case FR_DISK_ERR: return "disk I/O error";
    case FR_INT_ERR: return "assertion failed";
    case FR_NOT_READY: return "drive not ready";
    case FR_NO_FILE: return "file not found";
    case FR_NO_PATH: return "path not found";
    case FR_INVALID_NAME: return "invalid name";
    case FR_DENIED: return "access denied";
    case FR_EXIST: return "already exists";
    case FR_INVALID_OBJECT: return "invalid object";
    case FR_WRITE_PROTECTED: return "write protected";
    case FR_INVALID_DRIVE: return "invalid drive";
    case FR_NOT_ENABLED: return "not enabled";
    case FR_NO_FILESYSTEM: return "no filesystem";
    case FR_MKFS_ABORTED: return "mkfs aborted";
    case FR_TIMEOUT: return "timeout";
    case FR_LOCKED: return "locked";
    case FR_NOT_ENOUGH_CORE: return "out of memory";
    case FR_TOO_MANY_OPEN_FILES: return "too many open files";
    case FR_INVALID_PARAMETER: return "invalid parameter";
    default: return "unknown error";
    }
}

static void sdcard_power_on(void)
{
#ifdef BOARD_APP_SDCARD_POWER_EN_PIN
    HPM_IOC->PAD[BOARD_APP_SDCARD_POWER_EN_PIN].FUNC_CTL = 0;
    HPM_IOC->PAD[BOARD_APP_SDCARD_POWER_EN_PIN].PAD_CTL =
        IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
    const uint8_t on = BOARD_APP_SDCARD_POWER_EN_ACTIVE_HIGH ? 1 : 0;
    gpio_set_pin_output_with_initial(HPM_GPIO0,
                                     GPIO_GET_PORT_INDEX(BOARD_APP_SDCARD_POWER_EN_PIN),
                                     GPIO_GET_PIN_INDEX(BOARD_APP_SDCARD_POWER_EN_PIN),
                                     on);
    // Allow regulator / card to settle
    hpmhal.scheduler->delay(50);
#endif
}

/*
  Mount microSD. Safe to call from IO thread; must not block boot forever.
 */
bool sdcard_init(void)
{
    WITH_SEMAPHORE(sem);

    if (sdcard_running) {
        return true;
    }

    sdcard_power_on();
    board_init_sd_pins(BOARD_APP_SDCARD_SDXC_BASE);

    DSTATUS dstatus = disk_initialize(DEV_SD);
    if (dstatus != RES_OK) {
        hpmhal.console->printf("SD: disk_initialize failed (0x%x)\n", (unsigned)dstatus);
        disk_deinitialize(DEV_SD);
        return false;
    }

    FRESULT fres = f_mount(&s_sd_disk, sd_root, 1);
    if (fres == FR_NO_FILESYSTEM) {
        hpmhal.console->printf("SD: no FAT filesystem (format card on PC)\n");
        disk_deinitialize(DEV_SD);
        return false;
    }
    if (fres != FR_OK) {
        hpmhal.console->printf("SD: f_mount failed: %s\n", show_error_string(fres));
        disk_deinitialize(DEV_SD);
        return false;
    }

    fres = f_chdrive(sd_root);
    if (fres != FR_OK) {
        hpmhal.console->printf("SD: f_chdrive failed: %s\n", show_error_string(fres));
        f_unmount(sd_root);
        disk_deinitialize(DEV_SD);
        return false;
    }

    sdcard_running = true;
    hpmhal.console->printf("SD: mounted\n");
    return true;
}

void mount_sdcard()
{
    (void)sdcard_retry();
}

bool sdcard_retry(void)
{
    if (sdcard_running) {
        return true;
    }
    if (!sdcard_init()) {
        return false;
    }
#if AP_FILESYSTEM_FILE_WRITING_ENABLED
    // Outside sdcard_init semaphore — AP::FS uses its own recursive mutex
    AP::FS().mkdir("/APM");
    AP::FS().mkdir("/APM/LOGS");
#endif
    return true;
}

void unmount_sdcard()
{
    WITH_SEMAPHORE(sem);
    if (!sdcard_running) {
        return;
    }
    f_unmount(sd_root);
    disk_deinitialize(DEV_SD);
    sdcard_running = false;
}

void sdcard_stop(void)
{
    unmount_sdcard();
}

#else
void mount_sdcard() {}
void unmount_sdcard() {}
void sdcard_stop() {}
bool sdcard_retry(void) { return true; }
#endif
