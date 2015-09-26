/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "nvic.h"

#include "system.h"
#include "gpio.h"
#include "bus_spi.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_spi_mpu6500.h"
#include "gyro_sync.h"

enum lpf_e {
    INV_FILTER_256HZ_NOLPF2 = 0,
    INV_FILTER_188HZ,
    INV_FILTER_98HZ,
    INV_FILTER_42HZ,
    INV_FILTER_20HZ,
    INV_FILTER_10HZ,
    INV_FILTER_5HZ,
    INV_FILTER_2100HZ_NOLPF,
    NUM_FILTER
};

enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
};

enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
};

enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};

#define DISABLE_MPU6500       GPIO_SetBits(MPU6500_CS_GPIO,   MPU6500_CS_PIN)
#define ENABLE_MPU6500        GPIO_ResetBits(MPU6500_CS_GPIO, MPU6500_CS_PIN)

static uint8_t mpuLowPassFilter = INV_FILTER_42HZ;

static void mpu6500AccInit(void);
static bool mpu6500AccRead(int16_t *accData);
static void mpu6500GyroInit(void);
static bool mpu6500GyroRead(int16_t *gyroADC);
static void checkMPU6500Interrupt(bool *gyroIsUpdated);
void checkMPU6500DataReady(bool *mpuDataReadyPtr);

static bool mpuDataReady;
static const mpu6500Config_t *mpu6500Config = NULL;
extern uint16_t acc_1G;

void MPU_INTHandler(void)
{
    if (EXTI_GetITStatus(mpu6500Config->exti_line) == RESET) {
        return;
    }

    EXTI_ClearITPendingBit(mpu6500Config->exti_line);

    mpuDataReady = true;

#ifdef DEBUG_MPU_DATA_READY_INTERRUPT
    // Measure the delta in micro seconds between calls to the interrupt handler
    static uint32_t lastCalledAt = 0;
    static int32_t callDelta = 0;

    uint32_t now = micros();
    callDelta = now - lastCalledAt;

    //UNUSED(callDelta);
    debug[0] = callDelta;

    lastCalledAt = now;
#endif
}

void configureMPU6500DataReadyInterruptHandling(void)
{
#ifdef USE_MPU_DATA_READY_SIGNAL

#ifdef STM32F10X
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

#ifdef STM32F303xC
    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif

#ifdef STM32F40_41xxx
    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif

#ifdef STM32F10X
    gpioExtiLineConfig(mpu6500Config->exti_port_source, mpu6500Config->exti_pin_source);
#endif

#ifdef STM32F303xC
    gpioExtiLineConfig(mpu6500Config->exti_port_source, mpu6500Config->exti_pin_source);
#endif

#ifdef STM32F40_41xxx
    gpioExtiLineConfig(mpu6500Config->exti_port_source, mpu6500Config->exti_pin_source);
#endif

#ifdef ENSURE_MPU_DATA_READY_IS_LOW
    uint8_t status = GPIO_ReadInputDataBit(mpu6500Config->gpioPort, mpu6000Config->gpioPin);
    if (status) {
        return;
    }
#endif

    EXTI_ClearITPendingBit(mpu6500Config->exti_line);

    EXTI_InitTypeDef EXTIInit;
    EXTIInit.EXTI_Line = mpu6500Config->exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = mpu6500Config->exti_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_MPU_DATA_READY);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_MPU_DATA_READY);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
}

void mpu6500GpioInit(void) {
    gpio_config_t gpio;

    static bool mpu6500GpioInitDone = false;

    if (mpu6500GpioInitDone || !mpu6500Config) {
        return;
    }

#ifdef STM32F303
        if (mpu6500Config->gpioAHBPeripherals) {
            RCC_AHBPeriphClockCmd(mpu6500Config->gpioAHBPeripherals, ENABLE);
        }
#endif
#ifdef STM32F10X
        if (mpu6500Config->gpioAPB2Peripherals) {
            RCC_APB2PeriphClockCmd(mpu6500Config->gpioAPB2Peripherals, ENABLE);
        }
#endif
#ifdef STM32F40_41xxx
        if (mpu6500Config->gpioAHB1Peripherals) {
            RCC_AHB1PeriphClockCmd(mpu6500Config->gpioAHB1Peripherals, ENABLE);
        }
#endif

    gpio.pin = mpu6500Config->gpioPin;
    gpio.speed = Speed_2MHz;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(mpu6500Config->gpioPort, &gpio);

    configureMPU6500DataReadyInterruptHandling();

    mpu6500GpioInitDone = true;
}
//


static void mpu6500WriteRegister(uint8_t reg, uint8_t data)
{
    ENABLE_MPU6500;
    spiTransferByte(MPU6500_SPI_INSTANCE, reg);
    spiTransferByte(MPU6500_SPI_INSTANCE, data);
    DISABLE_MPU6500;
}

static void mpu6500ReadRegister(uint8_t reg, uint8_t *data, int length)
{
    ENABLE_MPU6500;
    spiTransferByte(MPU6500_SPI_INSTANCE, reg | 0x80); // read transaction
    spiTransfer(MPU6500_SPI_INSTANCE, data, NULL, length);
    DISABLE_MPU6500;
}

static void mpu6500SpiInit(void)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return;
    }

#ifdef STM32F303xC
    RCC_AHBPeriphClockCmd(MPU6500_CS_GPIO_CLK_PERIPHERAL, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = MPU6500_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(MPU6500_CS_GPIO, &GPIO_InitStructure);
#endif

#ifdef STM32F10X
    RCC_APB2PeriphClockCmd(MPU6500_CS_GPIO_CLK_PERIPHERAL, ENABLE);

    gpio_config_t gpio;
    // CS as output
    gpio.mode = Mode_Out_PP;
    gpio.pin = MPU6500_CS_PIN;
    gpio.speed = Speed_50MHz;
    gpioInit(MPU6500_CS_GPIO, &gpio);
#endif

    GPIO_SetBits(MPU6500_CS_GPIO,   MPU6500_CS_PIN);

    spiSetDivisor(MPU6500_SPI_INSTANCE, SPI_9MHZ_CLOCK_DIVIDER);

    hardwareInitialised = true;
}

static bool mpu6500Detect(void)
{
    uint8_t tmp;

    mpu6500SpiInit();

    mpu6500ReadRegister(MPU6500_RA_WHOAMI, &tmp, 1);

    if (tmp != MPU6500_WHO_AM_I_CONST)
        return false;

    return true;
}

bool mpu6500SpiAccDetect(const mpu6500Config_t *configToUse, acc_t *acc)
{
    mpu6500Config = configToUse;

    if (!mpu6500Detect()) {
        return false;
    }

    acc->init = mpu6500AccInit;
    acc->read = mpu6500AccRead;

    return true;
}

bool mpu6500SpiGyroDetect(const mpu6500Config_t *configToUse, gyro_t *gyro, uint16_t lpf)
{
    mpu6500Config = configToUse;

    if (!mpu6500Detect()) {
        return false;
    }

    gyro->init = mpu6500GyroInit;
    gyro->read = mpu6500GyroRead;
#ifdef USE_MPU_DATA_READY_SIGNAL
    gyro->intStatus = checkMPU6500DataReady;
#else
    gyro->intStatus = checkMPU6500Interrupt;
#endif

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;
    //gyro->scale = (4.0f / 16.4f) * (M_PIf / 180.0f) * 0.000001f;

    // default lpf is 42Hz
    if (lpf >= 188)
        mpuLowPassFilter = INV_FILTER_188HZ;
    else if (lpf >= 98)
        mpuLowPassFilter = INV_FILTER_98HZ;
    else if (lpf >= 42)
        mpuLowPassFilter = INV_FILTER_42HZ;
    else if (lpf >= 20)
        mpuLowPassFilter = INV_FILTER_20HZ;
    else if (lpf >= 10)
        mpuLowPassFilter = INV_FILTER_10HZ;
    else
        mpuLowPassFilter = INV_FILTER_5HZ;

    return true;
}

static void mpu6500AccInit(void)
{
    mpu6500GpioInit();
    acc_1G = 512 * 8;
}

static bool mpu6500AccRead(int16_t *accData)
{
    uint8_t buf[6];

    mpu6500ReadRegister(MPU6500_RA_ACCEL_XOUT_H, buf, 6);

    accData[X] = (int16_t)((buf[0] << 8) | buf[1]);
    accData[Y] = (int16_t)((buf[2] << 8) | buf[3]);
    accData[Z] = (int16_t)((buf[4] << 8) | buf[5]);

    return true;
}

static void mpu6500GyroInit(void)
{
#ifdef NAZE
    gpio_config_t gpio;
    // MPU_INT output on rev5 hardware (PC13). rev4 was on PB13, conflicts with SPI devices
    if (hse_value == 12000000) {
        gpio.pin = Pin_13;
        gpio.speed = Speed_2MHz;
        gpio.mode = Mode_IN_FLOATING;
        gpioInit(GPIOC, &gpio);
    }
#endif
    mpu6500GpioInit();

    mpu6500WriteRegister(MPU6500_RA_PWR_MGMT_1, MPU6500_BIT_RESET);
    delay(100);
    mpu6500WriteRegister(MPU6500_RA_PWR_MGMT_1, 0);
    delay(100);
    mpu6500WriteRegister(MPU6500_RA_PWR_MGMT_1, INV_CLK_PLL);
    mpu6500WriteRegister(MPU6500_RA_GYRO_CFG, INV_FSR_2000DPS << 3);
    mpu6500WriteRegister(MPU6500_RA_ACCEL_CFG, INV_FSR_8G << 3);
    mpu6500WriteRegister(MPU6500_RA_LPF, mpuLowPassFilter);
    mpu6500WriteRegister(MPU6500_RA_RATE_DIV, gyroMPU6xxxGetDividerDrops()); // Get Divider drop count
    mpu6500WriteRegister(MPU6500_RA_INT_ENABLE, 0x01);
}

static bool mpu6500GyroRead(int16_t *gyroADC)
{
    uint8_t buf[6];

    mpu6500ReadRegister(MPU6500_RA_GYRO_XOUT_H, buf, 6);

    gyroADC[X] = (int16_t)((buf[0] << 8) | buf[1]);
    gyroADC[Y] = (int16_t)((buf[2] << 8) | buf[3]);
    gyroADC[Z] = (int16_t)((buf[4] << 8) | buf[5]);

    return true;
}

#ifdef USE_MPU_DATA_READY_SIGNAL
void checkMPU6500DataReady(bool *mpuDataReadyPtr) {
    if (mpuDataReady) {
        *mpuDataReadyPtr = true;
        mpuDataReady= false;
    } else {
        *mpuDataReadyPtr = false;
    }
}
#else
void checkMPU6500Interrupt(bool *gyroIsUpdated) {
	uint8_t mpuIntStatus;

	mpu6500ReadRegister(MPU6500_RA_INT_STATUS, &mpuIntStatus, 1);

    if (mpuIntStatus) {
        *gyroIsUpdated = true;
    } else {
        *gyroIsUpdated = false;
    }
}
#endif

