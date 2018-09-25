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

#include <atmel_start.h>
#include "atmel_start.h"
#include "utils.h"
#include "nv.h"
#include "string.h"
#include "usb_protocol.h"
#include "hpl_wdt.h "
#include "nv_storage.h"
#include "driver_init.h"
#include "cdcdf_acm.h"
#include "cdcdf_acm_desc.h"
#include "uip.h"
#include "uip_arp.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hpl_adc_base.h>

// *****************************************************************************************************************************************************************
// Function:    bash_spi_transfer(unsigned char *tx, unsigned char *rx, int size)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: rolls round the push-pull MOSI/MISO of the SPI bus for [size] bytes of 8-bits, MSB
// Returns:     Nothing
// *****************************************************************************************************************************************************************
static void inline bash_spi_transfer(unsigned char *tx, unsigned char *rx, int size)
{
	int count, loop;
	
	for (count = 0; count < size; count++) {
		rx[count] = 0;
		
		for (loop = 8; loop; loop--) {
			gpio_set_pin_level(PB12_SPI_MOSI, (tx[count] & (1 << (loop - 1))) ? 1 : 0);
			gpio_set_pin_level(PB15_SPI_CLK, 0);
			gpio_set_pin_level(PB15_SPI_CLK, 1);
			
			rx[count] |= (gpio_get_pin_level(PB13_SPI_MISO) << (loop - 1));
		}
	}
}

// *****************************************************************************************************************************************************************
// Function:    EEprom_settings(unsigned char *data, unsigned int size, unsigned char write_notread)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: when requiring EEprom configuration data to be read or written to/from the non-volatile area, this facilitates the [NV] access
// Returns:     Nothing
// *****************************************************************************************************************************************************************
int EEprom_settings(unsigned char *data, unsigned int size, unsigned char write_notread)
{
	/* If we have a write flag set, then write the setting configuration overlay image into NV ram */
	if (write_notread) {
		return flash_write(&FLASH_0, 256 * 1024, data, size);
	}
	
	/* else, if we have a read flag set, then read the setting configuration overlay image from the NV ram into the "data" pointer"*/
	return flash_read(&FLASH_0, 256 * 1024, data, size);
}

static unsigned char	command_dataw[3 + 1], command_datar[3 + 1];

// *****************************************************************************************************************************************************************
// Function:    checkKSZreg(uint16_t reg, unsigned char verify)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: check a value in an SPI register of the KSZ-Micrel Ethernet SWITCH
// Returns:     does the register value match the check value? True: False
// *****************************************************************************************************************************************************************
bool checkKSZreg(uint16_t reg, unsigned char verify)
{
	/* Lower the nCS line for this SPI device */	
	gpio_set_pin_level(PB07_SPInCS_KSZ8974, 0);	
	
    /* Setup the 3 control bytes to perform a register read operation */		
	command_dataw[0] = 0x40 + REG_SPI_READ + (unsigned char)(reg >> 7);
	command_dataw[1] = (unsigned char)(reg & 0x7F) << 1;
	command_dataw[2] = 0x00;
	
	/* Perform the actual 3-byte push/pull SPI operation */
	bash_spi_transfer(command_dataw, command_datar, 3);

	/* Raise the nCS line for this SPI device */	
	gpio_set_pin_level(PB07_SPInCS_KSZ8974, 1);
	
	/* Check to see if the read data byte from the SPI end-point register matches the "verify" parameter and return the result as a boolean */
	return (command_datar[2] == verify) ? 1 : 0;
}

// *****************************************************************************************************************************************************************
// Function:    readKSZreg(uint16_t reg)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: read an SPI register of the KSZ-Micrel Ethernet SWITCH
// Returns:     value of register of KSZ Ethernet SWITCH SPI read
// *****************************************************************************************************************************************************************
unsigned char readKSZreg(uint16_t reg)
{
	/* Lower the nCS line for this SPI device */	
	gpio_set_pin_level(PB07_SPInCS_KSZ8974, 0);
	
	/* Setup the 3 control bytes to perform a register read operation */
	command_dataw[0] = 0x40 + REG_SPI_READ + (unsigned char)(reg >> 7);
	command_dataw[1] = (unsigned char)(reg & 0x7F) << 1;
	command_dataw[2] = 0x00;
	
	/* Perform the actual 3-byte push/pull SPI operation */
	bash_spi_transfer(command_dataw, command_datar, 3);
	
	/* Raise the nCS line for this SPI device */
	gpio_set_pin_level(PB07_SPInCS_KSZ8974, 1);
	
	/* Just return the actual far-end register data value */
	return (command_datar[2]);
}

// *****************************************************************************************************************************************************************
// Function:    writeKSZreg(uint16_t reg, unsigned char value)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: write to a SPI register of the KSZ-Micrel Ethernet SWITCH
// Returns:     nothing
// *****************************************************************************************************************************************************************
void writeKSZreg(uint16_t reg, unsigned char value)
{
	/* Lower the nCS line for this SPI device */
	gpio_set_pin_level(PB07_SPInCS_KSZ8974, 0);
	
	/* Setup the 3 control bytes to perform a register write operation */
	command_dataw[0] = 0x40 + REG_SPI_WRITE + (unsigned char)(reg >> 7);
	command_dataw[1] = (unsigned char)(reg & 0x7F) << 1;
	command_dataw[2] = value;
	
	/* Perform the actual 3-byte push/pull SPI operation */
	bash_spi_transfer(command_dataw, command_datar, 3);
	
	/* Raise the nCS line for this SPI device */
	gpio_set_pin_level(PB07_SPInCS_KSZ8974, 1);
}	

// *****************************************************************************************************************************************************************
// Function:    application_appcall(void)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: STUB for compilation - NOT USED
// Returns:     Nothing (NULL)
// *****************************************************************************************************************************************************************
void application_appcall(void)
{
	/* This is a stub function. It should not ever get called */
	xprintf("MultiSense application_appcall()\r\n");
}

// *****************************************************************************************************************************************************************
// Function:    application_init(void)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: Called by the uIP stack upon initialization to allow the application to set-up variables
// Returns:     Nothing (NULL)
// *****************************************************************************************************************************************************************
void application_init(void)
{
	/* This is a stub function. It should not ever get called */
	xprintf("MultiSense application_init()\r\n");
}

// *****************************************************************************************************************************************************************
// Function:    tcpip_output(void)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: STUB for compilation - NOT USED
// Returns:     Nothing (NULL)
// *****************************************************************************************************************************************************************
void tcpip_output(void)
{
	/* This is a stub function. It should not ever get called */
	xprintf("MultiSense tcpip_output()\r\n");
}

// *****************************************************************************************************************************************************************
// Function:    uip_log(char *msg)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: Called by the uIP stack to print out debug information, warnings and error messages
// Returns:     Nothing (NULL)
// *****************************************************************************************************************************************************************
void uip_log(char *msg)
{
	/* Print out the messages from the uIP stack */
	xprintf("MultiSense uIP log --> [%s]\r\n", msg);
}