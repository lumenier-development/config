/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define FC_TARGET_MCU     STM32H723

#define BOARD_NAME        MATEKH7A3
#define MANUFACTURER_ID   MTKS

#define USE_ACC
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO
#define USE_SPI_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_BARO
#define USE_BARO_BMP280
#define USE_MAX7456
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAG
#define USE_RX_MSP
#define USE_GPS

#define BEEPER_PIN           PB9

#define MOTOR1_PIN           PA9
#define MOTOR2_PIN           PA10
#define MOTOR3_PIN           PA15
#define MOTOR4_PIN           PB3
#define MOTOR5_PIN           PB0
#define MOTOR6_PIN           PB1
#define MOTOR7_PIN           PB4
#define MOTOR8_PIN           PB5
#define SERVO1_PIN           PB6
#define SERVO2_PIN           PB7
#define SERVO3_PIN           PB8

#define UART1_TX_PIN         PB14
#define UART1_RX_PIN         PB15
#define UART2_TX_PIN         PA2
#define UART2_RX_PIN         PA3
#define UART3_TX_PIN         PC10
#define UART3_RX_PIN         PC11
#define UART4_TX_PIN         PA0
#define UART4_RX_PIN         PA1
#define UART5_TX_PIN         PC12
#define UART5_RX_PIN         PD2
#define UART6_TX_PIN         PC6
#define UART6_RX_PIN         PC7

#define I2C3_SCL_PIN         PA8
#define I2C3_SDA_PIN         PC9

#define LED0_PIN             PA14
#define LED1_PIN             PA13

#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB10
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PC2
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PC3

#define ADC_VBAT_PIN         PC0
#define ADC_CURR_PIN         PC1
#define ADC_EXTERNAL1_PIN    PA4  //ADC12  VBAT2
#define ADC_EXTERNAL2_PIN    PC5  //ADC12  CURR2

#define PINIO1_PIN           PC13
#define MAX7456_SPI_CS_PIN   PB2
#define GYRO_1_CS_PIN        PC4
#define FLASH_CS_PIN         PC15

/*
    PA9  -  TIM1_CH2
    PA10 -  TIM1_CH3
    PA15 -  TIM2_CH1
    PB3  -  TIM2_CH2
    PB0  -  TIM3_CH3
    PB1  -  TIM3_CH4
    PB4  -  TIM3_CH1
    PB5  -  TIM3_CH2
    PB6  -  TIM4_CH1
    PB7  -  TIM4_CH2
    PB8  -  TIM16_CH1
*/

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, MOTOR1_PIN, 1,  -1) \
    TIMER_PIN_MAP( 1, MOTOR2_PIN, 1,  -1) \
    TIMER_PIN_MAP( 2, MOTOR3_PIN, 1,  -1) \
    TIMER_PIN_MAP( 3, MOTOR4_PIN, 1,  -1) \
    TIMER_PIN_MAP( 4, MOTOR5_PIN, 2,  -1) \
    TIMER_PIN_MAP( 5, MOTOR6_PIN, 2,  -1) \
    TIMER_PIN_MAP( 6, MOTOR7_PIN, 1,  -1) \
    TIMER_PIN_MAP( 7, MOTOR8_PIN, 1,  -1) \
    TIMER_PIN_MAP( 8, SERVO1_PIN, 2,  10) \
    TIMER_PIN_MAP( 9, SERVO2_PIN, 2,  11) \
    TIMER_PIN_MAP(10, SERVO3_PIN, 1,  12) \

#define ADC1_DMA_OPT        8
#define ADC3_DMA_OPT        9
#define TIMUP1_DMA_OPT      0
#define TIMUP2_DMA_OPT      1
#define TIMUP3_DMA_OPT      2
#define TIMUP4_DMA_OPT      4

#define BARO_I2C_INSTANCE I2CDEV_3
#define MAG_I2C_INSTANCE I2CDEV_3

#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_VOLTAGE_METER_SCALE_DEFAULT 110
#define DEFAULT_CURRENT_METER_SCALE 250
#define BEEPER_INVERTED
#define BEEPER_PWM_HZ 2500

#define MAX7456_SPI_INSTANCE SPI1
#define GYRO_1_SPI_INSTANCE SPI1
#define FLASH_SPI_INSTANCE SPI2

#define PINIO1_BOX 40
