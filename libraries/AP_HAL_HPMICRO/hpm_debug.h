#pragma once

#include "hpm_uart_drv.h"
#include "hpm_ioc_regs.h"
#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

static int _hpm_dbg_inited;

static void hpm_dbg_init(void)
{
    if (_hpm_dbg_inited) return;
    HPM_IOC->PAD[IOC_PAD_PY06].FUNC_CTL = IOC_PY06_FUNC_CTL_UART0_TXD;
    HPM_IOC->PAD[IOC_PAD_PY07].FUNC_CTL = IOC_PY07_FUNC_CTL_UART0_RXD;
    HPM_PIOC->PAD[IOC_PAD_PY06].FUNC_CTL = PIOC_PY06_FUNC_CTL_SOC_PY_06;
    HPM_PIOC->PAD[IOC_PAD_PY07].FUNC_CTL = PIOC_PY07_FUNC_CTL_SOC_PY_07;
    clock_add_to_group(clock_uart0, 0);
    uart_config_t cfg = {0};
    uart_default_config(HPM_UART0, &cfg);
    cfg.baudrate = 115200;
    cfg.fifo_enable = false;
    cfg.src_freq_in_hz = clock_get_frequency(clock_uart0);
    uart_init(HPM_UART0, &cfg);
    _hpm_dbg_inited = 1;
}

static void hpm_dbg_putc(char c)
{
    if (!_hpm_dbg_inited) hpm_dbg_init();
    while (!uart_check_status(HPM_UART0, uart_stat_tx_slot_avail));
    uart_send_byte(HPM_UART0, (uint8_t)c);
    if (c == '\n') {
        while (!uart_check_status(HPM_UART0, uart_stat_tx_slot_avail));
        uart_send_byte(HPM_UART0, '\r');
    }
}

static void hpm_dbg_puts(const char *s)
{
    while (*s) hpm_dbg_putc(*s++);
}

#define HPM_DBG(msg) do { \
    hpm_dbg_init(); \
    hpm_dbg_puts("[HPM] " msg "\n"); \
} while(0)

#ifdef __cplusplus
}
#endif
