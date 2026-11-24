/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "RCOutput.h"
#include <AP_Math/AP_Math.h>
#include "board.h"
#include "hpm_ioc_regs.h"
#include "hpm_iomux.h"
#include "hpm_trgmmux_src.h"

using namespace HPMicro;

#define SERVO_DEFAULT_FREQ_HZ 50

static const AP_HAL::HAL& hpmhal = AP_HAL::get_HAL();

#ifdef HAL_HPM_RCOUT
struct outpin_info outputs_pins[] = {HAL_HPM_RCOUT};
#else
struct outpin_info outputs_pins[] = {};
#endif

#define MAX_CHANNELS ARRAY_SIZE(outputs_pins)
static_assert(MAX_CHANNELS < 16, "overrunning _pending");
static_assert(MAX_CHANNELS < 32, "overrunning bitfields");

RCOutput::pwm_group RCOutput::pwm_group_list[] = HAL_PWM_GROUPS;
RCOutput::pwm_chan RCOutput::pwm_chan_list[MAX_CHANNELS];
const uint8_t RCOutput::num_channels = MAX_CHANNELS;

/*
 * Per-channel HW map for fmuv6x_hpm (schematic FMU_CH1..8):
 *   CH1 PD00 PWM3.P2          CH2 PD01 PWM3.P3
 *   CH3 PD02 PWM3.P4          CH4 PC29 PWM3.P7
 *   CH5 PC04 TRGM1.P11←PWM1.8 CH6 PC05 TRGM1.P09←PWM1.9
 *   CH7 PC09 TRGM1.P08←PWM1.10 CH8 PC08 TRGM1.P10←PWM1.11
 *
 * pwm_port in hwdef must match the PWM channel index used below.
 */
struct chan_hw_map {
    uint8_t group_id;
    uint8_t pwm_index;
    uint8_t cmp_index;
    uint32_t pad_func;
    TRGM_Type *trgm;
    uint8_t trgm_out;
    uint32_t trgm_src;
};

#if defined(HAL_HPM_FMU_PWM_MAP)
static const chan_hw_map fmu_chan_map[8] = {
    /* CH1 */ { 0, 2,  0, IOC_PD00_FUNC_CTL_PWM3_P_2,   nullptr,   0,  0 },
    /* CH2 */ { 0, 3,  1, IOC_PD01_FUNC_CTL_PWM3_P_3,   nullptr,   0,  0 },
    /* CH3 */ { 0, 4,  2, IOC_PD02_FUNC_CTL_PWM3_P_4,   nullptr,   0,  0 },
    /* CH4 */ { 0, 7,  3, IOC_PC29_FUNC_CTL_PWM3_P_7,   nullptr,   0,  0 },
    /* CH5 */ { 1, 8,  8, IOC_PC04_FUNC_CTL_TRGM1_P_11, HPM_TRGM1, 11, HPM_TRGM1_INPUT_SRC_PWM1_CH8REF },
    /* CH6 */ { 1, 9,  9, IOC_PC05_FUNC_CTL_TRGM1_P_09, HPM_TRGM1, 9,  HPM_TRGM1_INPUT_SRC_PWM1_CH9REF },
    /* CH7 */ { 1, 10, 10, IOC_PC09_FUNC_CTL_TRGM1_P_08, HPM_TRGM1, 8,  HPM_TRGM1_INPUT_SRC_PWM1_CH10REF },
    /* CH8 */ { 1, 11, 11, IOC_PC08_FUNC_CTL_TRGM1_P_10, HPM_TRGM1, 10, HPM_TRGM1_INPUT_SRC_PWM1_CH11REF },
};
#define HAS_FMU_CHAN_MAP 1
#else
#define HAS_FMU_CHAN_MAP 0
#endif

void RCOutput::setup_trgm_route(pwm_chan &ch)
{
    if (ch.trgm == nullptr) {
        return;
    }
    trgm_enable_io_output(ch.trgm, 1U << ch.trgm_out);
    trgm_output_t cfg = {};
    cfg.invert = false;
    cfg.type = trgm_output_same_as_input;
    cfg.input = ch.trgm_src;
    trgm_output_config(ch.trgm, ch.trgm_out, &cfg);
}

void RCOutput::init()
{
    // Enable clocks for every unique PWM group
    for (uint8_t idx = 0; idx < ARRAY_SIZE(pwm_group_list); idx++) {
        pwm_group &group = pwm_group_list[idx];
        group.mcpwm_group_id = idx;
        group.current_mode = MODE_PWM_NORMAL;
        if (group.rc_frequency == 0) {
            group.rc_frequency = SERVO_DEFAULT_FREQ_HZ;
        }
        clock_add_to_group(group.clock, 0);
        pwm_stop_counter(group.base);

        memset(&group.config, 0, sizeof(group.config));
        pwm_get_default_pwm_config(group.base, &group.config);
        group.config.enable_output = true;
        group.config.dead_zone_in_half_cycle = 0;
        group.config.invert_output = false;

        const uint32_t clk = clock_get_frequency(group.clock);
        uint32_t reload = (clk / group.rc_frequency);
        if (reload > 0) {
            reload -= 1;
        }
        group.reload = reload;
        pwm_set_reload(group.base, 0, reload);
        pwm_set_start_count(group.base, 0, 0);
    }

    for (uint8_t chan = 0; chan < MAX_CHANNELS; chan++) {
        pwm_chan &channel = pwm_chan_list[chan];
        memset(&channel, 0, sizeof(channel));

        uint8_t group_id = 0;
        uint8_t pwm_index = outputs_pins[chan].pwm_port_index;
        uint8_t cmp_index = chan;
        uint32_t pad_func = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(16);
        TRGM_Type *trgm = nullptr;
        uint8_t trgm_out = 0;
        uint32_t trgm_src = 0;

#if HAS_FMU_CHAN_MAP
        if (chan < ARRAY_SIZE(fmu_chan_map)) {
            const chan_hw_map &m = fmu_chan_map[chan];
            group_id = m.group_id;
            pwm_index = m.pwm_index;
            cmp_index = m.cmp_index;
            pad_func = m.pad_func;
            trgm = m.trgm;
            trgm_out = m.trgm_out;
            trgm_src = m.trgm_src;
        }
#endif
        if (group_id >= ARRAY_SIZE(pwm_group_list)) {
            hpmhal.console->printf("RCOUT: bad group for ch%u\n", (unsigned)chan);
            continue;
        }

        pwm_group &group = pwm_group_list[group_id];
        channel.group = &group;
        channel.gpio_num = outputs_pins[chan].pin_index;
        channel.port_num = pwm_index;
        channel.cmp_idx = cmp_index;
        channel.pad_func = pad_func;
        channel.trgm = trgm;
        channel.trgm_out = trgm_out;
        channel.trgm_src = trgm_src;
        channel.value_us = 0;
        channel.enabled = true;

        group.ch_mask |= (1U << chan);

        HPM_IOC->PAD[channel.gpio_num].FUNC_CTL = channel.pad_func;
        setup_trgm_route(channel);

        memset(&channel.cmp_config, 0, sizeof(channel.cmp_config));
        channel.cmp_config.mode = pwm_cmp_mode_output_compare;
        channel.cmp_config.cmp = group.reload + 1; // 0% duty until first write
        channel.cmp_config.update_trigger = pwm_shadow_register_update_on_modify;

        if (status_success != pwm_setup_waveform(group.base,
                                                 channel.port_num,
                                                 &group.config,
                                                 channel.cmp_idx,
                                                 &channel.cmp_config,
                                                 1)) {
            hpmhal.console->printf("RCOUT: waveform fail ch%u pwm=%u cmp=%u\n",
                                   (unsigned)chan,
                                   (unsigned)channel.port_num,
                                   (unsigned)channel.cmp_idx);
            channel.enabled = false;
            continue;
        }
    }

    for (uint8_t idx = 0; idx < ARRAY_SIZE(pwm_group_list); idx++) {
        pwm_group &group = pwm_group_list[idx];
        pwm_start_counter(group.base);
        pwm_issue_shadow_register_lock_event(group.base);
    }

    _corked = false;
    memset(_pending, 0, sizeof(_pending));
    _initialized = true;
    hpmhal.console->printf("RCOUT: init %u ch (%u groups)\n",
                           (unsigned)MAX_CHANNELS,
                           (unsigned)ARRAY_SIZE(pwm_group_list));
}

void RCOutput::set_group_mode(pwm_group &group)
{
    if (!_initialized) {
        return;
    }
    switch (group.current_mode) {
    case MODE_PWM_BRUSHED:
    case MODE_PWM_NONE:
    case MODE_PWM_NORMAL:
        break;
    case MODE_PWM_ONESHOT:
    case MODE_PWM_ONESHOT125:
        // TODO: oneshot trigger path
        break;
    default:
        group.current_mode = MODE_PWM_NORMAL;
        break;
    }
}

void RCOutput::set_output_mode(uint32_t mask, const enum output_mode mode)
{
    while (mask) {
        uint8_t chan = __builtin_ffs(mask) - 1;
        if (!_initialized || chan >= MAX_CHANNELS) {
            return;
        }
        if (pwm_chan_list[chan].group == nullptr) {
            mask &= ~(1U << chan);
            continue;
        }
        pwm_group &group = *pwm_chan_list[chan].group;
        group.current_mode = mode;
        set_group_mode(group);
        for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
            if (pwm_chan_list[i].group == &group) {
                mask &= ~(1U << i);
            }
        }
    }
}

void RCOutput::trigger_groups()
{
    rcout_timer_t now = rcout_micros();
    if (!AP_HAL::timeout_expired(last_pulse_trigger_us, now, trigger_widest_pulse)) {
        hpmhal.scheduler->delay_microseconds(
            AP_HAL::timeout_remaining(last_pulse_trigger_us, now, trigger_widest_pulse));
    }
    last_pulse_trigger_us = rcout_micros();
}

void RCOutput::timer_tick(void)
{
    if (!_initialized || last_pulse_trigger_us == 0) {
        return;
    }
    if (AP_HAL::timeout_expired(last_pulse_trigger_us, rcout_micros(),
                                trigger_widest_pulse + 4000U)) {
        trigger_groups();
    }
}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    if (!_initialized || freq_hz == 0) {
        return;
    }
    for (auto &group : pwm_group_list) {
        if ((group.ch_mask & chmask) == 0) {
            continue;
        }
        group.rc_frequency = freq_hz;
        const uint32_t clk = clock_get_frequency(group.clock);
        uint32_t reload = clk / freq_hz;
        if (reload > 0) {
            reload -= 1;
        }
        group.reload = reload;
        pwm_stop_counter(group.base);
        pwm_set_reload(group.base, 0, reload);
        pwm_set_start_count(group.base, 0, 0);
        pwm_start_counter(group.base);
        pwm_issue_shadow_register_lock_event(group.base);
    }
}

uint16_t RCOutput::get_freq(uint8_t chan)
{
    if (!_initialized || chan >= MAX_CHANNELS || pwm_chan_list[chan].group == nullptr) {
        return SERVO_DEFAULT_FREQ_HZ;
    }
    return pwm_chan_list[chan].group->rc_frequency;
}

void RCOutput::enable_ch(uint8_t chan)
{
    if (chan < MAX_CHANNELS) {
        pwm_chan_list[chan].enabled = true;
    }
}

void RCOutput::disable_ch(uint8_t chan)
{
    if (chan < MAX_CHANNELS) {
        pwm_chan_list[chan].enabled = false;
        write_channel(chan, 0);
    }
}

void RCOutput::cork(void)
{
    _corked = true;
}

void RCOutput::push(void)
{
    if (!_corked) {
        return;
    }
    _corked = false;
    for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
        write_channel(i, _pending[i]);
    }
}

void RCOutput::write_channel(uint8_t chan, uint16_t period_us)
{
    if (!_initialized || chan >= MAX_CHANNELS) {
        return;
    }
    pwm_chan &ch = pwm_chan_list[chan];
    if (ch.group == nullptr || !ch.enabled) {
        return;
    }
    pwm_group &group = *ch.group;
    ch.value_us = period_us;

    float duty = 0.0f;
    switch (group.current_mode) {
    case MODE_PWM_BRUSHED: {
        if (period_us <= _esc_pwm_min) {
            duty = 0.0f;
        } else if (period_us >= _esc_pwm_max) {
            duty = 100.0f;
        } else {
            duty = 100.0f * (float)(period_us - _esc_pwm_min) /
                   (float)(_esc_pwm_max - _esc_pwm_min);
        }
        break;
    }
    case MODE_PWM_NORMAL:
    case MODE_PWM_NONE:
    default: {
        const uint32_t period_us_total = 1000000U / group.rc_frequency;
        if (period_us_total > 0) {
            duty = 100.0f * (float)period_us / (float)period_us_total;
        }
        if (duty > 100.0f) {
            duty = 100.0f;
        }
        break;
    }
    }
    pwm_update_duty_edge_aligned(group.base, ch.cmp_idx, duty);
}

void RCOutput::write(uint8_t chan, uint16_t period_us)
{
    if (chan >= MAX_CHANNELS) {
        return;
    }
    _pending[chan] = period_us;
    if (!_corked) {
        write_channel(chan, period_us);
    }
}

uint16_t RCOutput::read(uint8_t chan)
{
    if (chan >= MAX_CHANNELS || !_initialized) {
        return 0;
    }
    return pwm_chan_list[chan].value_us;
}

void RCOutput::read(uint16_t* period_us, uint8_t len)
{
    for (uint8_t i = 0; i < MIN(len, MAX_CHANNELS); i++) {
        period_us[i] = read(i);
    }
}
