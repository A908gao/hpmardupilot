/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * This board that does not contain any sensors (but pins are active), it is a great help for a novice user,
 * by flashing an empty board, you can connect via Mavlink (Mission Planner - MP) and gradually add sensors.
 * If you had some sensor configured and it doesn't work then the MP connection does not work and then you may not know what to do next.
*/

#pragma once

#define TRUE  1
#define FALSE 0

//Protocols
// list of protocols/enum:  ardupilot/libraries/AP_SerialManager/AP_SerialManager.h
// default protocols:    ardupilot/libraries/AP_SerialManager/AP_SerialManager.cpp
// HPMicro serials:    AP_HAL_HPMICRO/HAL_HPM_Class.cpp

// Inertial sensors - ICM45686 on SPI2 (20-bit hi-res + CLKIN from OSC1 32.768kHz)
#ifndef HAL_INS_HIGHRES_SAMPLE
#define HAL_INS_HIGHRES_SAMPLE 1
#endif
#ifndef HAL_DEFAULT_INS_FAST_SAMPLE
#define HAL_DEFAULT_INS_FAST_SAMPLE 1
#endif
#define HAL_INS_DEFAULT HAL_INS_NONE
#define PROBE_IMU_SPI(driver, devname, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname),##args))
#define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensensev3, HAL_INS_ICM45686_NAME, ROTATION_YAW_180)

// Barometer - ICP20100 on I2C2 (hwdef bus 0). Probe both ADDR straps.
#define AP_BARO_ICP201XX_ENABLED 1
#define HAL_BARO_ALLOW_INIT_NO_BARO 1
#define HAL_BARO_PROBE_EXT_DEFAULT 1
#define HAL_BARO_PROBE_LIST \
    ADD_BACKEND(AP_Baro_ICP201XX::probe(*this, GET_I2C_DEVICE(0, 0x63))); \
    ADD_BACKEND(AP_Baro_ICP201XX::probe(*this, GET_I2C_DEVICE(0, 0x64)))

// Compass - external IIS2MDC on GPS1 I2C (hwdef bus 1 = I2C0 @ PB10/PB11)
#define AP_COMPASS_ENABLE_DEFAULT 1
#define AP_COMPASS_IIS2MDC_ENABLED 1
/* Explicit probe only — skip full external I2C mag sweep on GPS1 bus */
#undef HAL_PROBE_EXTERNAL_I2C_COMPASSES
#define ALLOW_ARM_NO_COMPASS
#define PROBE_MAG_I2C(driver, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(GET_I2C_DEVICE(bus, addr),##args))
/* bus 1 = external I2C0; 0x1E = IIS2MDC 7-bit addr; force_external=true */
#define HAL_MAG_PROBE_LIST PROBE_MAG_I2C(IIS2MDC, 1, HAL_COMPASS_IIS2MDC_I2C_ADDR, true, HAL_COMPASS_IIS2MDC_ORIENTATION_EXTERNAL)

// PWM groups for FMU_CH1-8:
//   CH1-4: PWM3 direct (P2/P3/P4/P7) on clock_mot3
//   CH5-8: PWM1 ch8-11 via TRGM1 (P11/P09/P08/P10) on clock_mot1
#define HAL_PWM_GROUPS {\
    { .mcpwm_group_id = 0, .base = HPM_PWM3, .clock = clock_mot3, .rc_frequency = 50, .ch_mask = 0x0F, }, \
    { .mcpwm_group_id = 1, .base = HPM_PWM1, .clock = clock_mot1, .rc_frequency = 50, .ch_mask = 0xF0, }, \
}
#define HAL_HPM_FMU_PWM_MAP 1

#define NUM_SERVO_CHANNELS    8

#define AP_BATT_MONITOR_MAX_INSTANCES 3
// GPIO36
#define HAL_BATT_VOLT_PIN (0)
#define HAL_BATT_VOLT_SCALE (18.1)
#define HAL_BATT2_VOLT_PIN (1)
#define HAL_BATT2_VOLT_SCALE (18.1)
//GPIO 32
#define HAL_BATT_CURR_PIN (2)
#define HAL_BATT_CURR_SCALE (36)
#define HAL_BATT2_CURR_PIN (3)
#define HAL_BATT2_CURR_SCALE (36)
//ADC
// #define HAL_DISABLE_ADC_DRIVER 1
#ifndef HAL_USE_ADC
#define HAL_USE_ADC 1
#endif

// LED - RGB on PB12/PB09/PB05 (HAL GPIO pin numbers from hwdef)
// Active-low like Pixhawk6X (MCU sinks LED cathode); leave HAL_GPIO_LED_ON at default 0.
#define AP_NOTIFY_GPIO_LED_RGB_ENABLED 1
#define AP_NOTIFY_GPIO_LED_RGB_RED_PIN 2
#define AP_NOTIFY_GPIO_LED_RGB_GREEN_PIN 3
#define AP_NOTIFY_GPIO_LED_RGB_BLUE_PIN 4
/*
 * Board RGB only by default. Probing NCP5623 on internal (baro) I2C and
 * sharing GPS1 I2C with IIS2MDC was associated with USB/MAVLink lockups.
 * Enable External NCP5623 via NTF_LED_TYPES once I2C bus sharing is solid.
 * Target value when stable: 225 = Board|DroneCAN|NCP5623_Ext|NCP5623_Int.
 */
#define DEFAULT_NTF_LED_TYPES Notify_LED_Board
// #define AP_FEATURE_BOARD_DETECT 1
/* Keep other I2C mag drivers off; IIS2MDC enabled explicitly above */
#define AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED 0
#define AP_BARO_BACKEND_DEFAULT_ENABLED 0
/* string names for well known SPI devices */
#define HAL_BARO_MS5611_NAME "ms5611"
#ifndef HAL_BARO_MS5611_SPI_INT_NAME
#define HAL_BARO_MS5611_SPI_INT_NAME "ms5611_int"
#endif
#define HAL_BARO_MS5611_SPI_EXT_NAME "ms5611_ext"
#define HAL_BARO_LPS22H_NAME "lps22h"
#define HAL_BARO_BMP280_NAME "bmp280"

#define HAL_INS_MPU60x0_NAME "mpu6000"
#define HAL_INS_MPU60x0_EXT_NAME "mpu6000_ext"

#define HAL_INS_LSM9DS0_G_NAME "lsm9ds0_g"
#define HAL_INS_LSM9DS0_A_NAME "lsm9ds0_am"

#define HAL_INS_LSM9DS0_EXT_G_NAME "lsm9ds0_ext_g"
#define HAL_INS_LSM9DS0_EXT_A_NAME "lsm9ds0_ext_am"

#define HAL_INS_MPU9250_NAME "mpu9250"
#define HAL_INS_MPU9250_EXT_NAME "mpu9250_ext"

#define HAL_INS_MPU6500_NAME "mpu6500"

#define HAL_INS_ICM45686_NAME "icm45686"
#define HAL_INS_ICM20608_NAME "icm20608"
#define HAL_INS_ICM20608_AM_NAME "icm20608-am"
#define HAL_INS_ICM20608_EXT_NAME "icm20608_ext"

// disable all frames for sim on hw except quad to save DRAM .bss
