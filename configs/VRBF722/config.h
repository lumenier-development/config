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

#define FC_TARGET_MCU   STM32F7X2

#define BOARD_NAME        VRBF722
#define MANUFACTURER_ID   VRBA

#define USE_ACC
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO
#define USE_SPI_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_FLASH
#define USE_BARO
#define USE_BARO_DPS310
#define USE_FLASH_W25Q128FV
#define USE_MAX7456
#define USE_EXTI
#define USE_MPU_DATA_READY_SIGNAL
#define USE_GYRO_EXTI
#define USE_MAG
#define USE_RX_MSP
#define USE_GPS
#define USE_LED_STRIP
#define USE_SERVOS

#define BEEPER_PIN           PC15

#define MOTOR1_PIN           PB0
#define MOTOR2_PIN           PB1
#define MOTOR3_PIN           PB5
#define MOTOR4_PIN           PB4
#define MOTOR5_PIN           PB6
#define MOTOR6_PIN           PB7
#define MOTOR7_PIN           PC8
#define MOTOR8_PIN           PC9
#define SERVO1_PIN           PA8

#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PB10
#define UART4_TX_PIN         PA0
// #define UART5_TX_PIN         PC12
#define UART6_TX_PIN         PC6

#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define UART3_RX_PIN         PB11
#define UART4_RX_PIN         PA1
#define UART5_RX_PIN         PD2
#define UART6_RX_PIN         PC7

#define I2C1_SCL_PIN         PB8
#define I2C1_SDA_PIN         PB9

#define LED0_PIN             PC14
// #define LED_STRIP_PIN        PA8

#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PC10

#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI3_SDI_PIN         PC11

#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PB15
#define SPI3_SDO_PIN         PC12

#define CAMERA_CONTROL_PIN   PB3
#define ADC_VBAT_PIN         PC2
// #define ADC_RSSI_PIN         PC0
#define ADC_CURR_PIN         PC1

#define PINIO1_PIN           PC13
#define FLASH_CS_PIN         PA15
#define MAX7456_SPI_CS_PIN   PB12
#define GYRO_1_EXTI_PIN      PC4
#define GYRO_1_CS_PIN        PA4
#define USB_DETECT_PIN       PB2

/*
    PB0 - TIM3_CH3 - DMA(1, 7, 5)
    PB1 - TIM3_CH4 - DMA(1, 2, 5)
    PB5 - TIM3_CH2 - DMA(1, 5, 5)
    PB4 - TIM3_CH1 - DMA(1, 4, 5)
    PB6 - TIM4_CH1 - DMA(1, 0, 2)
    PB7 - TIM4_CH2 - DMA(1, 3, 2)
    PC8 - TIM8_CH3 - DMA(2, 4, 7)
    PC9 - TIM8_CH4 - DMA(2, 7, 7)
    PB3 - TIM2_CH2 - DMA(1, 6, 3)
    PA3 - TIM5_CH4 - DMA(1, 3, 6)
    PA8 - TIM1_CH1 - DMA(2, 1, 6)
*/

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP(0,  PB0,  2,  0) \
    TIMER_PIN_MAP(1,  PB1,  2,  0) \
    TIMER_PIN_MAP(2,  PB5,  1,  0) \
    TIMER_PIN_MAP(3,  PB4,  1,  0) \
    TIMER_PIN_MAP(4,  PB6,  1,  0) \
    TIMER_PIN_MAP(5,  PB7,  1,  0) \
    TIMER_PIN_MAP(6,  PC8,  2,  1) \
    TIMER_PIN_MAP(7,  PC9,  2,  0) \
    TIMER_PIN_MAP(8,  PB3,  1,  0) \
    TIMER_PIN_MAP(9,  PA3,  2,  1) \
    TIMER_PIN_MAP(10,  PA8,  1,  1) \
    
#define ADC1_DMA_OPT        0

#define MAG_I2C_INSTANCE            (I2CDEV_1)
#define BARO_I2C_INSTANCE           (I2CDEV_1)
#define DEFAULT_BARO_I2C_ADDRESS    118

#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_VOLTAGE_METER_SCALE_DEFAULT 110
#define DEFAULT_CURRENT_METER_SCALE 85
#define BEEPER_INVERTED

#define DEFAULT_GYRO_TO_USE GYRO_CONFIG_USE_GYRO_1
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN_YAW 1800
#define FLASH_SPI_INSTANCE SPI3
#define MAX7456_SPI_INSTANCE SPI2

#define PINIO1_CONFIG 129
#define PINIO1_BOX 0