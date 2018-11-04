/*
 * The Clear BSD License
 * Copyright 2017-2018 NXP
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

/*! @name PORTB18 (number 41), J1[1]/I2S_TX_BCLK/LCD_P14
  @{ */
#define BOARD_LCD_P14_PORT PORTB /*!<@brief PORT device name: PORTB */
#define BOARD_LCD_P14_PIN 18U    /*!<@brief PORTB pin index: 18 */
                                 /* @} */

/*! @name PORTB19 (number 42), J1[3]/I2S_TX_FS/LCD_P15
  @{ */
#define BOARD_LCD_P15_PORT PORTB /*!<@brief PORT device name: PORTB */
#define BOARD_LCD_P15_PIN 19U    /*!<@brief PORTB pin index: 19 */
                                 /* @} */

/*! @name PORTC0 (number 43), J1[5]/I2S_TXD/LCD_P20
  @{ */
#define BOARD_LCD_P20_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_LCD_P20_PIN 0U     /*!<@brief PORTC pin index: 0 */
                                 /* @} */

/*! @name PORTC4 (number 53), J1[7]/I2S_MCLK/LCD_P24
  @{ */
#define BOARD_LCD_P24_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_LCD_P24_PIN 4U     /*!<@brief PORTC pin index: 4 */
                                 /* @} */

/*! @name PORTC6 (number 55), J1[9]/I2S_RX_BCLK/LCD_P26
  @{ */
#define BOARD_LCD_P26_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_LCD_P26_PIN 6U     /*!<@brief PORTC pin index: 6 */
                                 /* @} */

/*! @name PORTC7 (number 56), J1[11]/I2S_RX_FS/USB_SOF_OUT/LCD_P27
  @{ */
#define BOARD_LCD_P27_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_LCD_P27_PIN 7U     /*!<@brief PORTC pin index: 7 */
                                 /* @} */

/*! @name PORTD0 (number 57), LCD_P40
  @{ */
#define BOARD_LCD_P40_PORT PORTD /*!<@brief PORT device name: PORTD */
#define BOARD_LCD_P40_PIN 0U     /*!<@brief PORTD pin index: 0 */
                                 /* @} */

/*! @name PORTD2 (number 59), J2[4]/D9/LCD_P42
  @{ */
#define BOARD_LCD_P42_PORT PORTD /*!<@brief PORT device name: PORTD */
#define BOARD_LCD_P42_PIN 2U     /*!<@brief PORTD pin index: 2 */
                                 /* @} */

/*! @name PORTD3 (number 60), J1[6]/D2/LCD_P43
  @{ */
#define BOARD_LCD_P43_PORT PORTD /*!<@brief PORT device name: PORTD */
#define BOARD_LCD_P43_PIN 3U     /*!<@brief PORTD pin index: 3 */
                                 /* @} */

/*! @name PORTD4 (number 61), J2[6]/D10/SPI1_PCS0/LCD_P44
  @{ */
#define BOARD_LCD_P44_PORT PORTD /*!<@brief PORT device name: PORTD */
#define BOARD_LCD_P44_PIN 4U     /*!<@brief PORTD pin index: 4 */
                                 /* @} */

/*! @name PORTE20 (number 9), J4[1]/DIFF_ADC0_DP/LCD_P59
  @{ */
#define BOARD_LCD_P59_PORT PORTE /*!<@brief PORT device name: PORTE */
#define BOARD_LCD_P59_PIN 20U    /*!<@brief PORTE pin index: 20 */
                                 /* @} */

/*! @name PORTE21 (number 10), J4[3]/DIFF_ADC0_DM/LCD_P60
  @{ */
#define BOARD_LCD_P60_PORT PORTE /*!<@brief PORT device name: PORTE */
#define BOARD_LCD_P60_PIN 21U    /*!<@brief PORTE pin index: 21 */
                                 /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitLCD(void);

/*! @name PORTA4 (number 26), J1[10]/D4/SW1
  @{ */
#define BOARD_SW1_GPIO GPIOA /*!<@brief GPIO device name: GPIOA */
#define BOARD_SW1_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_SW1_PIN 4U     /*!<@brief PORTA pin index: 4 */
                             /* @} */

/*! @name PORTC3 (number 46), SW3/LLWU_P7/LCD_P23
  @{ */
#define BOARD_SW3_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
#define BOARD_SW3_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_SW3_PIN 3U     /*!<@brief PORTC pin index: 3 */
                             /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitBUTTONS(void);

/*! @name PORTE31 (number 19), LED2
  @{ */
#define BOARD_LED2_GPIO GPIOE /*!<@brief GPIO device name: GPIOE */
#define BOARD_LED2_PORT PORTE /*!<@brief PORT device name: PORTE */
#define BOARD_LED2_PIN 31U    /*!<@brief PORTE pin index: 31 */
                              /* @} */

/*! @name PORTD5 (number 62), J2[12]/D13/SPI1_SCK/LED1/LCD_P45
  @{ */
#define BOARD_LED1_GPIO GPIOD /*!<@brief GPIO device name: GPIOD */
#define BOARD_LED1_PORT PORTD /*!<@brief PORT device name: PORTD */
#define BOARD_LED1_PIN 5U     /*!<@brief PORTD pin index: 5 */
                              /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitLEDs(void);

#define SOPT5_LPUART0RXSRC_LPUART_RX 0x00u /*!<@brief LPUART0 Receive Data Source Select: LPUART_RX pin */
#define SOPT5_LPUART0TXSRC_LPUART_TX 0x00u /*!<@brief LPUART0 Transmit Data Source Select: LPUART0_TX pin */

/*! @name PORTA1 (number 23), J1[2]/D0/UART0_RX
  @{ */
#define BOARD_DEBUG_UART0_RX_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_DEBUG_UART0_RX_PIN 1U     /*!<@brief PORTA pin index: 1 */
                                        /* @} */

/*! @name PORTA2 (number 24), J1[4]/D1/UART0_TX
  @{ */
#define BOARD_DEBUG_UART0_TX_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_DEBUG_UART0_TX_PIN 2U     /*!<@brief PORTA pin index: 2 */
                                        /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitDEBUG_UART(void);

/*! @name USB0_DP (number 5), J10[3]/USB_DP
  @{ */
/* @} */

/*! @name USB0_DM (number 6), J10[2]/USB_DM
  @{ */
/* @} */

/*! @name VREGIN (number 8), KL43Z_REGIN
  @{ */
/* @} */

/*! @name PORTC0 (number 43), J1[5]/I2S_TXD/LCD_P20
  @{ */
#define BOARD_USB_SOF_OUT_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_USB_SOF_OUT_PIN 0U     /*!<@brief PORTC pin index: 0 */
                                     /* @} */

/*! @name PORTA5 (number 27), J1[12]/D5/I2S_TX_BCLK
  @{ */
#define BOARD_I2S0_TX_BCLK_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_I2S0_TX_BCLK_PIN 5U     /*!<@brief PORTA pin index: 5 */
                                      /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitUSB(void);

/*! @name PORTE24 (number 20), U2[7]/U10[4]/I2C0_SCL
  @{ */
#define BOARD_I2C0_SCL_PORT PORTE /*!<@brief PORT device name: PORTE */
#define BOARD_I2C0_SCL_PIN 24U    /*!<@brief PORTE pin index: 24 */
                                  /* @} */

/*! @name PORTE25 (number 21), U2[6]/U10[6]/I2C0_SDA
  @{ */
#define BOARD_I2C1_SDA_PORT PORTE /*!<@brief PORT device name: PORTE */
#define BOARD_I2C1_SDA_PIN 25U    /*!<@brief PORTE pin index: 25 */
                                  /* @} */

/*! @name PORTD1 (number 58), U2[9]/U10[9]/INT2_ACCEL/INT1_MAG/LCD_P41
  @{ */
#define BOARD_INT2_ACCEL_GPIO GPIOD /*!<@brief GPIO device name: GPIOD */
#define BOARD_INT2_ACCEL_PORT PORTD /*!<@brief PORT device name: PORTD */
#define BOARD_INT2_ACCEL_PIN 1U     /*!<@brief PORTD pin index: 1 */
                                    /* @} */

/*! @name PORTC5 (number 54), J1[15]/I2S_RXD/INT1_ACCEL/LCD_P25
  @{ */
#define BOARD_INT1_ACCEL_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
#define BOARD_INT1_ACCEL_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INT1_ACCEL_PIN 5U     /*!<@brief PORTC pin index: 5 */
                                    /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitACCEL_I2C(void);

/*! @name PORTA18 (number 32), EXTAL_32KHZ
  @{ */
#define BOARD_EXTAL_32KHZ_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_EXTAL_32KHZ_PIN 18U    /*!<@brief PORTA pin index: 18 */
                                     /* @} */

/*! @name PORTA19 (number 33), XTAL_32KHZ
  @{ */
#define BOARD_XTAL_32KHZ_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_XTAL_32KHZ_PIN 19U    /*!<@brief PORTA pin index: 19 */
                                    /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitOSC(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
