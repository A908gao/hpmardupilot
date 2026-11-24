/*
 * Lightweight IOMCU link probe for HPM FreeRTOS boards.
 * Speaks the px4io/ArduPilot IOPacket register protocol on UART5 @ 1.5 Mbit
 * without enabling HAL_WITH_IO_MCU / AP_IOMCU.
 *
 * Probe I/O runs on a FreeRTOS task; GCS reporting is done from the main
 * thread via service() (GCS is not safe from arbitrary tasks).
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef HAL_HPM_IOMCU_PROBE
#define HAL_HPM_IOMCU_PROBE 0
#endif

#if HAL_HPM_IOMCU_PROBE

namespace HPMicro
{

class IOMCUProbe
{
public:
    static void start(void);
    // Call from main thread to emit pending STATUSTEXT.
    static void service(void);

private:
    static void probe_task(void *arg);
    static void thread(void);
    static bool wait_bytes(AP_HAL::UARTDriver &uart, uint16_t n, uint32_t timeout_ms);
    static bool read_page_config(AP_HAL::UARTDriver &uart, uint16_t *regs, uint8_t count, bool short_hdr);
};

}

#endif // HAL_HPM_IOMCU_PROBE
