/*
 * Lightweight IOMCU link probe for HPM FreeRTOS boards.
 * Uses a private UART5 (uart_desc[0]) at 1.5 Mbps and PAGE_CONFIG.
 * Does not enable HAL_WITH_IO_MCU / AP_IOMCU.
 */
#include "IOMCUProbe.h"

#if HAL_HPM_IOMCU_PROBE

#include "UARTDriver.h"
#include "Scheduler.h"
#include <AP_IOMCU/iofirmware/ioprotocol.h>
#include <AP_Math/crc.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <string.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"

extern const AP_HAL::HAL &hal;

namespace HPMicro
{

// Dedicated UART5 instance (uart_desc[0]); not owned by SerialManager.
static UARTDriver iomcu_uart(0);

enum class ReportKind : uint8_t {
    None = 0,
    Started,
    Ok,
    Timeout,
    ThreadFail,
};

struct Report {
    volatile ReportKind kind;
    volatile uint32_t attempt;
    volatile uint32_t avail;
    volatile uint16_t ver;
    volatile uint16_t ver2;
    volatile uint32_t mcuid;
    volatile bool short_mode;
};

static Report report {};

void IOMCUProbe::probe_task(void *arg)
{
    (void)arg;
    thread();
}

void IOMCUProbe::start(void)
{
    // Stack depth is in words on this FreeRTOS port (same units as Scheduler::*_SS).
    if (xTaskCreate(probe_task, "iomcu_probe", 4096, nullptr, Scheduler::IO_PRIO, nullptr) != pdPASS) {
        report.kind = ReportKind::ThreadFail;
    }
}

void IOMCUProbe::service(void)
{
    const ReportKind kind = report.kind;
    if (kind == ReportKind::None) {
        return;
    }
    report.kind = ReportKind::None;

    switch (kind) {
    case ReportKind::Started:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IOMCU probe: UART5 @ 1.5M");
        break;
    case ReportKind::Ok:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "IOMCU OK %s v%u/%u mcu=0x%08lx try=%lu",
                      report.short_mode ? "short" : "full",
                      unsigned(report.ver),
                      unsigned(report.ver2),
                      (unsigned long)report.mcuid,
                      (unsigned long)report.attempt);
        break;
    case ReportKind::Timeout:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "IOMCU timeout avail=%lu try=%lu",
                      (unsigned long)report.avail,
                      (unsigned long)report.attempt);
        break;
    case ReportKind::ThreadFail:
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IOMCU probe: thread create failed");
        break;
    case ReportKind::None:
        break;
    }
}

bool IOMCUProbe::wait_bytes(AP_HAL::UARTDriver &uart, uint16_t n, uint32_t timeout_ms)
{
    const uint32_t t0 = AP_HAL::millis();
    while (AP_HAL::millis() - t0 < timeout_ms) {
        uart._timer_tick();
        if (uart.available() >= n) {
            return true;
        }
        hal.scheduler->delay(1);
    }
    uart._timer_tick();
    return uart.available() >= n;
}

bool IOMCUProbe::read_page_config(AP_HAL::UARTDriver &uart, uint16_t *regs, uint8_t count, bool short_hdr)
{
    IOPacket pkt {};
    uart.discard_input();

    pkt.code = CODE_READ;
    pkt.count = count;
    pkt.page = PAGE_CONFIG;
    pkt.offset = 0;
    pkt.crc = 0;

    const uint8_t pkt_size = short_hdr ? 4 : pkt.get_size();
    pkt.crc = crc_crc8(reinterpret_cast<const uint8_t *>(&pkt), pkt_size);

    const size_t wrote = uart.write(reinterpret_cast<const uint8_t *>(&pkt), pkt_size);
    const uint32_t tw0 = AP_HAL::millis();
    while (uart.tx_pending() && AP_HAL::millis() - tw0 < 20) {
        uart._timer_tick();
        hal.scheduler->delay_microseconds(200);
    }
    if (wrote != pkt_size) {
        return false;
    }

    const uint16_t expect = uint16_t(count) * 2U + 4U;
    if (!wait_bytes(uart, expect, 50)) {
        return false;
    }

    const uint16_t n = uart.available();
    if (n < offsetof(IOPacket, regs) || n > sizeof(pkt)) {
        uart.discard_input();
        return false;
    }
    if (pkt.get_size() != n) {
        uart.discard_input();
        return false;
    }

    if (uart.read(reinterpret_cast<uint8_t *>(&pkt), n) != (ssize_t)n) {
        return false;
    }

    const uint8_t got_crc = pkt.crc;
    pkt.crc = 0;
    const uint8_t exp_crc = crc_crc8(reinterpret_cast<const uint8_t *>(&pkt), pkt.get_size());
    if (got_crc != exp_crc) {
        return false;
    }
    if (pkt.code != CODE_SUCCESS || pkt.count < count) {
        return false;
    }

    memcpy(regs, pkt.regs, count * 2U);
    return true;
}

void IOMCUProbe::thread(void)
{
    // Let USB CDC / MAVLink settle before poking the IO UART.
    hal.scheduler->delay(3000);
    report.kind = ReportKind::Started;

    iomcu_uart.begin(1500 * 1000, 256, 256);
    iomcu_uart.set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);

    uint32_t attempt = 0;

    while (true) {
        attempt++;
        page_config cfg {};
        uint16_t *regs = reinterpret_cast<uint16_t *>(&cfg);
        const uint8_t nregs = sizeof(cfg) / 2;

        bool ok = read_page_config(iomcu_uart, regs, nregs, true);
        bool short_mode = true;
        if (!ok) {
            ok = read_page_config(iomcu_uart, regs, nregs, false);
            short_mode = false;
        }

        if (ok) {
            report.attempt = attempt;
            report.ver = cfg.protocol_version;
            report.ver2 = cfg.protocol_version2;
            report.mcuid = cfg.mcuid;
            report.short_mode = short_mode;
            report.kind = ReportKind::Ok;
            hal.scheduler->delay(2000);
        } else {
            // Report every 5 tries to avoid STATUSTEXT spam.
            if ((attempt % 5) == 1) {
                report.attempt = attempt;
                report.avail = iomcu_uart.available();
                report.kind = ReportKind::Timeout;
                iomcu_uart.discard_input();
            }
            hal.scheduler->delay(500);
        }
    }
}

}

#endif // HAL_HPM_IOMCU_PROBE
