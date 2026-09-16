/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Parameter / mission storage. Prefer on-board SPI FRAM (FM25V02A) when
 * HAL_WITH_RAMTRON is enabled — same model as ChibiOS Pixhawk boards.
 */
#include <AP_HAL/AP_HAL.h>
#include "HAL_HPM_Class.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "Storage.h"
#include "FreeRTOS.h"

#if !HAL_WITH_RAMTRON
#ifndef HAL_HPM_STORAGE_FLASH_BASE_ADDR
#define HAL_HPM_STORAGE_FLASH_BASE_ADDR 0x80300000
#endif
#ifndef HAL_HPM_STORAGE_OFFSET_ADDR
#define HAL_HPM_STORAGE_OFFSET_ADDR 0x0300000
#endif
#endif

using namespace HPMicro;

static const AP_HAL::HAL& hpmhal = AP_HAL::get_HAL();

void Storage::_storage_open(void)
{
    if (_backend != StorageBackend::None) {
        return;
    }

    _dirty_mask.clearall();
    memset(_buffer, 0, sizeof(_buffer));

#if HAL_WITH_RAMTRON
    if (_fram.init() && _fram.read(0, _buffer, STORAGE_SIZE)) {
        _backend = StorageBackend::FRAM;
        _last_empty_ms = AP_HAL::millis();
        hpmhal.console->printf("Storage: FRAM OK (%u bytes)\n", (unsigned)STORAGE_SIZE);
        return;
    }
    hpmhal.console->printf("Storage: FRAM init failed, using RAM-only defaults\n");
    _backend = StorageBackend::RAMOnly;
    _last_empty_ms = AP_HAL::millis();
    return;
#else
    memset((void *)&_xpi_nor_config, 0, sizeof(_xpi_nor_config));
    xpi_nor_config_option_t option;
    option.header.U = BOARD_APP_XPI_NOR_CFG_OPT_HDR;
    option.option0.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT0;
    option.option1.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT1;

    XPI_Type *base = BOARD_APP_XPI_NOR_XPI_BASE;
    (void)rom_xpi_nor_auto_config(base, &_xpi_nor_config, &option);
    rom_xpi_nor_get_property(BOARD_APP_XPI_NOR_XPI_BASE, &_xpi_nor_config, xpi_nor_property_total_size, &_flash_size);
    rom_xpi_nor_get_property(BOARD_APP_XPI_NOR_XPI_BASE, &_xpi_nor_config, xpi_nor_property_sector_size, &_sector_size);
    rom_xpi_nor_get_property(BOARD_APP_XPI_NOR_XPI_BASE, &_xpi_nor_config, xpi_nor_property_page_size, &_page_size);
    _flash_load();
    _backend = StorageBackend::Flash;
    _last_empty_ms = AP_HAL::millis();
#endif
}

void Storage::_mark_dirty(uint16_t loc, uint16_t length)
{
    uint16_t end = loc + length;
    for (uint16_t line = loc >> STORAGE_LINE_SHIFT;
         line <= end >> STORAGE_LINE_SHIFT;
         line++) {
        _dirty_mask.set(line);
    }
}

void Storage::read_block(void *dst, uint16_t loc, size_t n)
{
    if (loc >= sizeof(_buffer) - (n - 1)) {
        return;
    }
    _storage_open();
    memcpy(dst, &_buffer[loc], n);
}

void Storage::write_block(uint16_t loc, const void *src, size_t n)
{
    if (loc >= sizeof(_buffer) - (n - 1)) {
        return;
    }
    if (memcmp(src, &_buffer[loc], n) != 0) {
        _storage_open();
        memcpy(&_buffer[loc], src, n);
        if (_backend != StorageBackend::RAMOnly) {
            _mark_dirty(loc, n);
        }
    }
}

void Storage::_timer_tick(void)
{
    if (_backend == StorageBackend::None || _backend == StorageBackend::RAMOnly) {
        return;
    }
    if (_dirty_mask.empty()) {
        _last_empty_ms = AP_HAL::millis();
        return;
    }

    uint16_t i;
    for (i = 0; i < STORAGE_NUM_LINES; i++) {
        if (_dirty_mask.get(i)) {
            break;
        }
    }
    if (i == STORAGE_NUM_LINES) {
        return;
    }

    memcpy(_tmpline, &_buffer[STORAGE_LINE_SIZE * i], STORAGE_LINE_SIZE);
    bool write_ok = false;

#if HAL_WITH_RAMTRON
    if (_backend == StorageBackend::FRAM) {
        write_ok = _fram.write(STORAGE_LINE_SIZE * i, _tmpline, STORAGE_LINE_SIZE);
    }
#endif

#if !HAL_WITH_RAMTRON
    if (_backend == StorageBackend::Flash) {
        if (_flash.write(i * STORAGE_LINE_SIZE, STORAGE_LINE_SIZE)) {
            write_ok = true;
        }
    }
#endif

    if (write_ok &&
        memcmp(_tmpline, &_buffer[STORAGE_LINE_SIZE * i], STORAGE_LINE_SIZE) == 0) {
        _dirty_mask.clear(i);
    }
}

bool Storage::healthy(void)
{
    if (_backend == StorageBackend::None) {
        return false;
    }
    if (_backend == StorageBackend::RAMOnly) {
        return true;
    }
    return (AP_HAL::millis() - _last_empty_ms < 2000u);
}

bool Storage::get_storage_ptr(void *&ptr, size_t &size)
{
    if (_backend == StorageBackend::None) {
        return false;
    }
    ptr = _buffer;
    size = sizeof(_buffer);
    return true;
}

#if !HAL_WITH_RAMTRON
void Storage::_flash_load(void)
{
    if (!_flash.init()) {
        AP_HAL::panic("unable to init flash storage");
    }
}

void Storage::_flash_write(uint16_t line)
{
    if (_flash.write(line * STORAGE_LINE_SIZE, STORAGE_LINE_SIZE)) {
        _dirty_mask.clear(line);
    }
}

bool Storage::_flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length)
{
    hpm_stat_t status;
    uint32_t *src = (uint32_t *)malloc(length + 4);
    uint32_t *src_aligned = (uint32_t *)(((uint32_t)src + 3) & 0xFFFFFFFC);
    if (src == nullptr) {
        return false;
    }
    memcpy(src_aligned, data, length);
    uint32_t address = sector * STORAGE_SECTOR_SIZE + offset + HAL_HPM_STORAGE_OFFSET_ADDR;
    l1c_dc_flush_all();
    portENTER_CRITICAL();
    status = rom_xpi_nor_program(BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &_xpi_nor_config, src_aligned, address, (uint32_t)length);
    __asm volatile ("fence.i");
    portEXIT_CRITICAL();
    free(src);
    return status == status_success;
}

bool Storage::_flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length)
{
    uint32_t address = sector * STORAGE_SECTOR_SIZE + offset + HAL_HPM_STORAGE_OFFSET_ADDR;
    return rom_xpi_nor_read(BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &_xpi_nor_config, (uint32_t *)data, address, length) == status_success;
}

bool Storage::_flash_erase_sector(uint8_t sector)
{
    uint32_t address = sector * STORAGE_SECTOR_SIZE + HAL_HPM_STORAGE_OFFSET_ADDR;
    portENTER_CRITICAL();
    hpm_stat_t status = rom_xpi_nor_erase_sector(BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &_xpi_nor_config, address);
    portEXIT_CRITICAL();
    return status == status_success;
}

bool Storage::_flash_erase_ok(void)
{
    return !hpmhal.util->get_soft_armed();
}
#endif
