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

#pragma once
#define TARGET_BOARD_IDENTIFIER "NUCL" // NUCLUES

#define LED0_GPIO   GPIOE
#define LED0_PIN    Pin_5 // Blue LED
#define LED0_PERIPHERAL RCC_AHB1Periph_GPIOE

#define LED1_GPIO   GPIOE
#define LED1_PIN    Pin_6  // Green LED
#define LED1_PERIPHERAL RCC_AHB1Periph_GPIOE

#define USABLE_TIMER_CHANNEL_COUNT 16

#define MPU6000_CS_GPIO       GPIOB
#define MPU6000_CS_PIN        GPIO_Pin_8
#define MPU6000_SPI_INSTANCE  SPI3

#define ACC
#define USE_ACC_SPI_MPU6000
#define ACC_SPI_MPU6000_ALIGN CW0_DEG

#define GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_SPI_MPU6000_ALIGN CW0_DEG

#define INVERTER_PIN Pin_12 
#define INVERTER_GPIO GPIOB
#define INVERTER_PERIPHERAL RCC_AHB1Periph_GPIOB
#define INVERTER_USART USART3

------Checked

#define MAG
//#define USE_MAG_HMC5883
#define HMC5883_BUS I2C_DEVICE_INT
#define MAG_HMC5883_ALIGN CW0_DEG

#define BARO
#define USE_BARO_MS5611
#define MS5611_BUS I2C_DEVICE_INT

#define INVERTER
#define LED0
#define LED1

#define SENSORS_SET (SENSOR_ACC|SENSOR_MAG|SENSOR_BARO)



#define AUTOTUNE
#define USE_CLI