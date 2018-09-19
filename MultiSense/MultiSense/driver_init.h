/* *****************************************************************************************************************************************************************
   (C) Copyright Detection Technologies Ltd / PeriNet GmbH 2018. All Rights Reserved
   - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
   Nine Ways Research & Development Ltd authorized on behalf of Detection / PeriNet
   Project: MultiSense / VibraTek / MaxiIO / MiniIO / Daughter Card IO
   Author:  Mr Paul P. Bates
   Company: Nine Ways Research & Development Ltd
   Date:    May/June 2018
   - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
   License: STRICTLY FOR DETECTION TECHNOLOGY LTD, PERINET GmbH, NINE WAYS R&D LTD. ANY UNAUTHORIZED COPYING OR DISTRIBUTION IS PROHIBITED
   File:    Part of the MultiSense Source Build in Atmel Studio for Microchip SAM Series CPU/SoC
   ***************************************************************************************************************************************************************** */

#ifndef DRIVER_INIT_INCLUDED
#define DRIVER_INIT_INCLUDED

#include "atmel_start_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>
#include <hal_adc_sync.h>
#include <hal_aes_sync.h>
#include <hal_crc_sync.h>
#include <hal_sha_sync.h>
#include <hal_flash.h>
#include <hal_usart_async.h>
#include <hal_spi_m_sync.h>
#include <hal_usart_async.h>
#include <hal_delay.h>
#include <hal_timer.h>
#include <hpl_tc_base.h>
#include "hal_usb_device.h"
#include <hal_wdt.h>
#include <hal_mac_async.h>

extern struct adc_sync_descriptor ADC_0;
extern struct aes_sync_descriptor CRYPTOGRAPHY_0;
extern struct crc_sync_descriptor CRC_0;
extern struct sha_sync_descriptor HASH_ALGORITHM_0;
extern struct flash_descriptor       FLASH_0;
extern struct usart_async_descriptor USART_0;
extern struct spi_m_sync_descriptor  SPI_0;
extern struct usart_async_descriptor USART_1;
extern struct timer_descriptor TIMER_0;
extern struct wdt_descriptor WDT_0;
extern struct mac_async_descriptor ETHERNET_MAC_0;

void ADC_0_PORT_init(void);
void ADC_0_CLOCK_init(void);
void ADC_0_init(void);
void FLASH_0_init(void);
void FLASH_0_CLOCK_init(void);
void USART_0_PORT_init(void);
void USART_0_CLOCK_init(void);
void USART_0_init(void);
void SPI_0_PORT_init(void);
void SPI_0_CLOCK_init(void);
void SPI_0_init(void);
void USART_1_PORT_init(void);
void USART_1_CLOCK_init(void);
void USART_1_init(void);
void delay_driver_init(void);
void USB_0_CLOCK_init(void);
void USB_0_init(void);
void WDT_0_CLOCK_init(void);
void WDT_0_init(void);
void ETHERNET_MAC_0_CLOCK_init(void);
void ETHERNET_MAC_0_init(void);
void ETHERNET_MAC_0_PORT_init(void);
void ETHERNET_MAC_0_example(void);

/**
 * \brief Perform system initialization, initialize pins and clocks for
 * peripherals
 */
void system_init(void);

#ifdef __cplusplus
}
#endif
#endif // DRIVER_INIT_INCLUDED
