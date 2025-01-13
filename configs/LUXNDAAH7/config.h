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

#define FC_TARGET_MCU     STM32H743

#define BOARD_NAME        LUXNDAAH7
#define MANUFACTURER_ID   LMNR

#define USE_ACC
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO
#define USE_SPI_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_FLASH
#define USE_BARO
#define USE_BARO_DPS310
#define USE_FLASH_W25Q128FV
#define USE_SDCARD
#define USE_MAX7456
#define USE_EXTI
#define USE_MPU_DATA_READY_SIGNAL
#define USE_GYRO_EXTI
#define USE_MAG
#define USE_RX_MSP
#define USE_GPS

#define BEEPER_PIN           PC8

#define MOTOR1_PIN           PA0
#define MOTOR2_PIN           PA1
#define MOTOR3_PIN           PA2
#define MOTOR4_PIN           PA3
#define MOTOR5_PIN           PD12
#define MOTOR6_PIN           PD13
#define MOTOR7_PIN           PD14
#define MOTOR8_PIN           PD15

#define RX_PPM_PIN           PC9

#define LED_STRIP_PIN        PB0

#define UART1_TX_PIN         PB14
#define UART2_TX_PIN         PD5
#define UART3_TX_PIN         PD8
#define UART4_TX_PIN         PD1
#define UART5_TX_PIN         PB13
#define UART6_TX_PIN         PC6
#define UART7_TX_PIN         PE8
#define UART8_TX_PIN         PE1

#define UART1_RX_PIN         PB15
#define UART2_RX_PIN         PD6
#define UART3_RX_PIN         PD9
#define UART4_RX_PIN         PD0
#define UART5_RX_PIN         PB12
#define UART6_RX_PIN         PC7
#define UART7_RX_PIN         PE7
#define UART8_RX_PIN         PE0

#define I2C1_SCL_PIN         PB6
#define I2C1_SDA_PIN         PB7
#define I2C2_SCL_PIN         PB10
#define I2C2_SDA_PIN         PB11

#define LED0_PIN             PE3

#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PA9
#define SPI3_SCK_PIN         PC10
#define SPI4_SCK_PIN         PE2
#define SPI6_SCK_PIN         PB3

#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PC2
#define SPI3_SDI_PIN         PC11
#define SPI4_SDI_PIN         PE5
#define SPI6_SDI_PIN         PB4

#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PC3
#define SPI3_SDO_PIN         PC12
#define SPI4_SDO_PIN         PE6
#define SPI6_SDO_PIN         PB5

#define CAMERA_CONTROL_PIN   PB1
#define ADC_VBAT_PIN         PC0
#define ADC_RSSI_PIN         PC5
#define ADC_CURR_PIN         PC1
#define ADC_EXTERNAL1_PIN    PC4  // Airspeed

#define SDCARD_SPI_CS_PIN    PA10 // SPI2_NSS
#define SDCARD_DETECT_PIN    PE12
#define PINIO1_PIN           PB2    // VTX Power Switch
#define FLASH_CS_PIN         PA4    // SPI1_NSS
#define MAX7456_SPI_CS_PIN   PE4    // SPI4_NSS
#define GYRO_1_EXTI_PIN      PC13  
#define GYRO_1_CS_PIN        PD7    // SPI6_NSS
#define USB_DETECT_PIN       PD2

/*
    PA0 - TIM2_CH1
    PA1 - TIM2_CH2
    PA2 - TIM2_CH3 
    PA3 - TIM2_CH4
    PD12 - TIM4_CH1
    PD13 - TIM4_CH2
    PD14 - TIM4_CH3
    PD15 - TIM4_CH4
    PA8 - TIM1_CH1
    PE11 - TIM1_CH2
    PE13 - TIM1_CH3 
    PE14 - TIM1_CH4
    PC9 - TIM8_CH4
    PB0 - TIM3_CH3
*/

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, MOTOR1_PIN,  1, -1) \
    TIMER_PIN_MAP( 1, MOTOR2_PIN,  1, -1) \
    TIMER_PIN_MAP( 2, MOTOR3_PIN,  1, -1) \
    TIMER_PIN_MAP( 3, MOTOR4_PIN,  1, -1) \
    TIMER_PIN_MAP( 4, MOTOR5_PIN,  1, -1) \
    TIMER_PIN_MAP( 5, MOTOR6_PIN,  1, -1) \
    TIMER_PIN_MAP( 6, MOTOR7_PIN,  1, -1) \
    TIMER_PIN_MAP( 7, MOTOR8_PIN,  1, -1) \
    TIMER_PIN_MAP( 8, PA8,   1, 10) \
    TIMER_PIN_MAP( 9, PE11,  1, 11) \
    TIMER_PIN_MAP( 10, PE13, 1, 12) \
    TIMER_PIN_MAP( 11, PE14, 1, 13) \
    TIMER_PIN_MAP( 12, PC9,  2, 14) \
    TIMER_PIN_MAP( 13, PB0,  2, 15) \

#define TIMUP2_DMA_OPT      0 
#define TIMUP4_DMA_OPT      1 
#define TIMUP1_DMA_OPT      2
#define ADC1_DMA_OPT        8
#define ADC3_DMA_OPT        9

#define USE_DSHOT_DMAR
#define DEFAULT_DSHOT_BURST DSHOT_DMAR_ON

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
#define GYRO_1_SPI_INSTANCE SPI6
#define GYRO_1_ALIGN_YAW 1800
#define FLASH_SPI_INSTANCE SPI1
#define MAX7456_SPI_INSTANCE SPI4
#define SDCARD_SPI_INSTANCE SPI2
#define SDCARD_DETECT_INVERTED

#define PINIO1_CONFIG 1
#define PINIO1_BOX 0