/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "I2CDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "Scheduler.h"
#include "Semaphores.h"
#include <stdio.h>
#include "hpm_gpio_drv.h"
#include "hpm_i2c_drv.h"
#include "hpm_soc.h"
#include "hpm_ioc_regs.h"
#include "board.h"

using namespace HPMicro;

static const AP_HAL::HAL& hpmhal = AP_HAL::get_HAL();
#define MHZ (1000U*1000U)
#define KHZ (1000U)

// #define I2CDEBUG 1
I2CBusDesc i2c_bus_desc[] = { HAL_HPM_I2C_BUSES };

I2CBus I2CDeviceManager::businfo[ARRAY_SIZE(i2c_bus_desc)];

static I2CDeviceManager *i2c_mgr_singleton;

I2CDeviceManager::I2CDeviceManager(void)
    : _initialized(false)
{
    i2c_mgr_singleton = this;
    // Hardware init must wait for board_init_clock() — see init().
}

void I2CDeviceManager::init(void)
{
    if (_initialized) {
        return;
    }
#ifdef I2CDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    for (uint8_t i=0; i<ARRAY_SIZE(i2c_bus_desc); i++) {
        if (i2c_bus_desc[i].soft) {
            while(1) {
                hpmhal.scheduler->delay(1000);
            }
        } else {
            HPM_IOC->PAD[i2c_bus_desc[i].sda].FUNC_CTL = i2c_bus_desc[i].sda_af | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
            HPM_IOC->PAD[i2c_bus_desc[i].scl].FUNC_CTL = i2c_bus_desc[i].scl_af | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
            if (i2c_bus_desc[i].sda_baf) {
                HPM_BIOC->PAD[i2c_bus_desc[i].sda].FUNC_CTL = i2c_bus_desc[i].sda_baf;
            }
            if (i2c_bus_desc[i].scl_baf) {
                HPM_BIOC->PAD[i2c_bus_desc[i].scl].FUNC_CTL = i2c_bus_desc[i].scl_baf;
            }
            if (i2c_bus_desc[i].sda_paf) {
                HPM_PIOC->PAD[i2c_bus_desc[i].sda].FUNC_CTL = i2c_bus_desc[i].sda_paf;
            }
            if (i2c_bus_desc[i].scl_paf) {
                HPM_PIOC->PAD[i2c_bus_desc[i].scl].FUNC_CTL = i2c_bus_desc[i].scl_paf;
            }

            // Open-drain + pull-up (board also has external pullups on I2C2)
            HPM_IOC->PAD[i2c_bus_desc[i].sda].PAD_CTL =
                IOC_PAD_PAD_CTL_OD_MASK | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
            HPM_IOC->PAD[i2c_bus_desc[i].scl].PAD_CTL =
                IOC_PAD_PAD_CTL_OD_MASK | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
            i2c_config_t config;
            hpm_stat_t stat;
            uint32_t freq;
            // Ensure peripheral clock is in the running group (same as board_init_i2c_clock)
            freq = board_init_i2c_clock(i2c_bus_desc[i].port);
            if (freq == 0) {
                clock_add_to_group(i2c_bus_desc[i].bus_clock, 0);
                freq = clock_get_frequency(i2c_bus_desc[i].bus_clock);
            }
            if (freq == 0) {
                businfo[i].port = nullptr;
                businfo[i].bus_clock = i2c_bus_desc[i].bus_clock;
                businfo[i].soft = false;
                continue;
            }
            config.i2c_mode = (i2c_bus_desc[i].speed >= 400*KHZ) ? i2c_mode_fast : i2c_mode_normal;
            config.is_10bit_addressing = false;
            stat = i2c_init_master(i2c_bus_desc[i].port, freq, &config);
            if (stat != status_success) {
                businfo[i].port = nullptr;
                businfo[i].bus_clock = i2c_bus_desc[i].bus_clock;
                businfo[i].soft = false;
                continue;
            }
            businfo[i].port = i2c_bus_desc[i].port;
            businfo[i].bus_clock = i2c_bus_desc[i].bus_clock;
            businfo[i].speed = i2c_bus_desc[i].speed;
            businfo[i].bus_num = i;
            businfo[i].soft = false;
        }
    }
    _initialized = true;
}

/*
 * Issue STOP after a failed transfer. CMPL timeouts in the HPM driver often
 * leave the controller busy without STOP; keep this minimal (no delay/re-init)
 * so probe paths cannot hang the boot before USB comes up.
 */
void I2CBus::recover(void)
{
    if (port == nullptr || soft) {
        return;
    }
    port->STATUS = I2C_STATUS_CMPL_MASK | I2C_STATUS_ADDRHIT_MASK;
    port->CTRL = I2C_CTRL_PHASE_STOP_MASK;
    port->CMD = I2C_CMD_ISSUE_DATA_TRANSMISSION;
    i2c_reset(port);
    i2c_config_t config;
    config.i2c_mode = (speed >= 400*KHZ) ? i2c_mode_fast : i2c_mode_normal;
    config.is_10bit_addressing = false;
    uint32_t freq = clock_get_frequency(bus_clock);
    if (freq != 0) {
        (void)i2c_init_master(port, freq, &config);
    }
}

void I2CDeviceManager::init_singleton(void)
{
    if (i2c_mgr_singleton != nullptr) {
        i2c_mgr_singleton->init();
    }
}

I2CDevice::I2CDevice(uint8_t busnum, uint8_t address, uint32_t bus_clock, bool use_smbus, uint32_t timeout_ms) :
    bus(I2CDeviceManager::businfo[busnum]),
    _retries(10),
    _address(address),
    _timeout_ms(timeout_ms)
{
#ifdef I2CDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    set_device_bus(busnum);
    set_device_address(address);
    pname = (char *)malloc(50);
    if (pname) {
        sprintf(pname, "I2C:%u:%02x",
                 (unsigned)busnum, (unsigned)address);
    }
}

I2CDevice::~I2CDevice()
{
#ifdef I2CDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    free(pname);
}

AP_HAL::OwnPtr<AP_HAL::I2CDevice>
I2CDeviceManager::get_device(uint8_t bus, uint8_t address,
                             uint32_t bus_clock,
                             bool use_smbus,
                             uint32_t timeout_ms)
{
    init_singleton();
    if (bus >= ARRAY_SIZE(i2c_bus_desc)) {
        return AP_HAL::OwnPtr<AP_HAL::I2CDevice>(nullptr);
    }
    auto dev = AP_HAL::OwnPtr<AP_HAL::I2CDevice>(NEW_NOTHROW I2CDevice(bus, address, bus_clock, use_smbus, timeout_ms));
    return dev;
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
#ifdef I2CDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    I2CDeviceManager::init_singleton();
    if (!bus.semaphore.check_owner()) {
        return false;
    }

    if (bus.soft || bus.port == nullptr) {
        return false;
    }

    const bool do_send = (send_len != 0 && send != nullptr);
    const bool do_recv = (recv_len != 0 && recv != nullptr);
    if (!do_send && !do_recv) {
        return false;
    }

    /* Cap retries: probe used to burn boot time and re-init thrash the bus */
    const uint8_t attempts = (_retries > 0) ? MIN(_retries, (uint8_t)3) : 1;
    for (uint8_t i = 0; i < attempts; i++) {
        hpm_stat_t st = status_invalid_argument;
        if (do_send && do_recv) {
            st = i2c_master_address_read(bus.port, _address,
                                         (uint8_t *)send, send_len,
                                         recv, recv_len);
        } else if (do_send) {
            st = i2c_master_write(bus.port, _address, (uint8_t *)send, send_len);
        } else {
            st = i2c_master_read(bus.port, _address, recv, recv_len);
        }
        if (st == status_success) {
            return true;
        }
        bus.recover();
    }
    return false;
}

/*
  register a periodic callback
*/
AP_HAL::Device::PeriodicHandle I2CDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
#ifdef I2CDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    return bus.register_periodic_callback(period_usec, cb, this);
}


/*
  adjust a periodic callback
*/
bool I2CDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
#ifdef I2CDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    return bus.adjust_timer(h, period_usec);
}

/*
  get mask of bus numbers for all configured I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask(void) const
{
#ifdef I2CDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    return ((1U << ARRAY_SIZE(i2c_bus_desc)) - 1);
}

/*
  get mask of bus numbers for all configured internal I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_internal(void) const
{
#ifdef I2CDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    uint32_t result = 0;
    for (size_t i = 0; i < ARRAY_SIZE(i2c_bus_desc); i++) {
        if (i2c_bus_desc[i].internal) {
            result |= (1u << i);
        }
    }
    return result;
}

/*
  get mask of bus numbers for all configured external I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_external(void) const
{
#ifdef I2CDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    return get_bus_mask() & ~get_bus_mask_internal();
}
