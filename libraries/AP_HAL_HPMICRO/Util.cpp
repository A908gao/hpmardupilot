/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "Util.h"
#include "RCOutput.h"
#include <AP_ROMFS/AP_ROMFS.h>
#include "sdcard.h"
#include <stdlib.h>
#include <string.h>
#include <AP_Common/ExpandingString.h>
#include <FreeRTOS.h>
#include "task.h"
#include "tinyalloc.h"
#include "hpm_ppor_drv.h"
#include "hpm_rtc_drv.h"
#include "hpm_soc_feature.h"

static const AP_HAL::HAL& hpmhal = AP_HAL::get_HAL();

using namespace HPMicro;

extern "C" {
    uint32_t otp_read_from_shadow(uint32_t addr);
}

/*
  Factory chip UID is OTP words 8..11 (16 bytes). UUID at OTP_SOC_UUID_IDX is
  a fallback. OTP MAC0 is often blank on production boards (shows as zeros).
 */
static bool read_chip_hwid(uint8_t out[16])
{
    uint32_t words[4] {};
    for (uint32_t i = 0; i < 4; i++) {
        words[i] = otp_read_from_shadow(8U + i);
    }
    const bool uid_blank = (words[0] | words[1] | words[2] | words[3]) == 0U;
    if (uid_blank) {
        for (uint32_t i = 0; i < 4; i++) {
            words[i] = otp_read_from_shadow(OTP_SOC_UUID_IDX + i);
        }
    }
    memcpy(out, words, 16);
    return (words[0] | words[1] | words[2] | words[3]) != 0U;
}
void Util::malloc_type_init(uint8_t *buffer, size_t size)
{
    bool ok  = ta_init(buffer, buffer + size, 256, 16, 8);
    if (!ok) {
        hpmhal.console->printf("tinyalloc init failed\n");
        while(1);
    }
}

/**
   how much free memory do we have in bytes.
*/
uint32_t Util::available_memory(void)
{
    return xPortGetFreeHeapSize();
}

/*
    Special Allocation Routines
*/

void* Util::malloc_type(size_t size, AP_HAL::Util::Memory_Type mem_type)
{
    void *p = nullptr;
    if (mem_type == AP_HAL::Util::MEM_DMA_SAFE) {
        p = ta_alloc(size);
        if (p == nullptr) {
            hpmhal.console->printf("Util: ta_alloc failed for %u bytes\n", (unsigned)size);
        }
        return p;
    }

    // FAST = normal/freeRTOS heap allocation
    if (mem_type == AP_HAL::Util::MEM_FAST) {
        p = pvPortMalloc(size);
        if (p == nullptr) {
            hpmhal.console->printf("Util: pvPortMalloc failed for %u bytes\n", (unsigned)size);
        }
        return p;
    }

    return (void *)NULL;
}

void Util::free_type(void *ptr, size_t size, AP_HAL::Util::Memory_Type mem_type)
{
    if (ptr == nullptr) {
        return;
    }

    if (mem_type == AP_HAL::Util::MEM_DMA_SAFE) {
        ta_free(ptr);
        return;
    }

    // FAST = normal/freeRTOS heap free
    if (mem_type == AP_HAL::Util::MEM_FAST) {
        vPortFree(ptr);
        return;
    }
}


/*
  get safety switch state
 */
Util::safety_state Util::safety_switch_state(void)
{
    return (AP_HAL::Util::safety_state)0;
}

/*
  set HW RTC in UTC microseconds
*/
void Util::set_hw_rtc(uint64_t time_utc_usec)
{
    // Convert microseconds to seconds since epoch
    time_t time_seconds = time_utc_usec / 1000000;
    // Configure RTC with the time in seconds
    rtc_config_time(HPM_RTC, time_seconds);
}

/*
  get system clock in UTC microseconds
*/
uint64_t Util::get_hw_rtc() const
{
    // Get time in seconds since epoch
    time_t time_seconds = rtc_get_time(HPM_RTC);
    // Convert to microseconds
    return static_cast<uint64_t>(time_seconds) * 1000000;
}

#if !defined(HAL_NO_FLASH_SUPPORT) && !defined(HAL_NO_ROMFS_SUPPORT)

#if !HAL_GCS_ENABLED
#define Debug(fmt, args ...)  do { hpmhal.console->printf(fmt, ## args); } while (0)
#else
#include <GCS_MAVLink/GCS.h>
#define Debug(fmt, args ...)  do { GCS_SEND_TEXT(MAV_SEVERITY_INFO, fmt, ## args); } while (0)
#endif

Util::FlashBootloader Util::flash_bootloader()
{
    return FlashBootloader::FAIL;
}
#endif // !HAL_NO_FLASH_SUPPORT && !HAL_NO_ROMFS_SUPPORT

/*
  display system identifier - board type and serial number
 */

bool Util::get_system_id(char buf[50])
{
    uint8_t uid[16] {};
    if (!read_chip_hwid(uid)) {
        snprintf(buf, 50, "HPMicro (no UID)");
        return false;
    }
    // 12 bytes fits "HPMicro " + spaced hex into 50 chars (STM32-style width)
    snprintf(buf, 50,
             "HPMicro %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X",
             uid[0], uid[1], uid[2], uid[3],
             uid[4], uid[5], uid[6], uid[7],
             uid[8], uid[9], uid[10], uid[11]);
    return true;
}

bool Util::get_system_id_unformatted(uint8_t buf[], uint8_t &len)
{
    uint8_t uid[16] {};
    if (!read_chip_hwid(uid)) {
        len = 0;
        return false;
    }
    len = MIN(len, (uint8_t)sizeof(uid));
    memcpy(buf, uid, len);
    return true;
}

// return true if the reason for the reboot was a watchdog reset
bool Util::was_watchdog_reset() const
{
    if ((ppor_reset_get_flags(HPM_PPOR) & ppor_reset_wdog0) || 
        (ppor_reset_get_flags(HPM_PPOR) & ppor_reset_wdog1) ||
        (ppor_reset_get_flags(HPM_PPOR) & ppor_reset_wdog2) ||
        (ppor_reset_get_flags(HPM_PPOR) & ppor_reset_wdog3) ||
        (ppor_reset_get_flags(HPM_PPOR) & ppor_reset_pmic_wdog)) {
        return true;
    }
    return false;
}

/*
  display stack usage as text buffer for @SYS/threads.txt
 */
void Util::thread_info(ExpandingString &str)
{
    const UBaseType_t ntasks = uxTaskGetNumberOfTasks();
    str.printf("tasks=%u heap_free=%u heap_min=%u heap_total=%u\n",
               unsigned(ntasks),
               unsigned(xPortGetFreeHeapSize()),
               unsigned(xPortGetMinimumEverFreeHeapSize()),
               unsigned(configTOTAL_HEAP_SIZE));
    str.printf("%-16s %5s %4s %10s %8s\n",
               "Name", "State", "Prio", "FreeBytes", "FreeWords");

    if (ntasks == 0 || ntasks > 64) {
        return;
    }
    TaskStatus_t status[64];
    configRUN_TIME_COUNTER_TYPE runtime = 0;
    const UBaseType_t got = uxTaskGetSystemState(status, ntasks, &runtime);
    for (UBaseType_t i = 0; i < got; i++) {
        const uint32_t free_b = uint32_t(status[i].usStackHighWaterMark) * sizeof(StackType_t);
        str.printf("%-16s %5u %4u %10u %8u\n",
                   status[i].pcTaskName ? status[i].pcTaskName : "?",
                   unsigned(status[i].eCurrentState),
                   unsigned(status[i].uxCurrentPriority),
                   unsigned(free_b),
                   unsigned(status[i].usStackHighWaterMark));
    }
}

void Util::mem_info(ExpandingString &str)
{
    str.printf("FreeRTOS heap\n");
    str.printf("  free:      %u\n", unsigned(xPortGetFreeHeapSize()));
    str.printf("  min_ever:  %u\n", unsigned(xPortGetMinimumEverFreeHeapSize()));
    str.printf("  total:     %u\n", unsigned(configTOTAL_HEAP_SIZE));
    str.printf("\n");
    mem_map_info(str);
}