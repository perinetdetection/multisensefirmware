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
#include "stdio.h"
#include "stdarg.h"
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

/* external function list */
extern int							I2C_configMONITOREDINPUTS(uint8_t,  uint8_t, unsigned char);
extern int							I2C_readMONITOREDINPUT(uint8_t,  uint8_t, unsigned char *, unsigned char *);
extern int							I2C_configEXPANDER(uint8_t,  uint8_t);
extern int							I2C_setEXPANDER(uint8_t,  uint8_t, unsigned char);
extern int							I2C_readEXPANDER(uint8_t,  uint8_t, unsigned char *);
extern int							I2C_setGAIN(uint8_t,  uint8_t, unsigned char);
extern int							I2C_check_deviceID(uint8_t, uint8_t, unsigned char);
extern int							I2C_getTEMPandMOISTURE(uint8_t,  uint8_t, unsigned char *, unsigned char *, unsigned char *, unsigned char *);
extern bool							checkKSZreg(uint16_t, unsigned char);
extern unsigned char				readKSZreg(uint16_t);
extern void							writeKSZreg(uint16_t, unsigned char);

/* external global variables list */
extern unsigned char	            readdata_tempmoisture[4], readdata_water1, readdata_water2, highvoltage;
extern unsigned char				readenvironment, arp_check, ip_periodic_check, sentA, sentB, reboot_actioned, refresh_gain;
extern unsigned char		 		settings_buffer[], ring, read_hardware_index;
extern unsigned int				    card_sampleindex;
extern unsigned char				tamper, cardA_present, cardB_present, cardA_old, cardB_old, good_ethernet, link_port1, link_port2, link_port3, ring_broken;
extern unsigned char	        	miniIO_A1_adcH, miniIO_A1_adcL, miniIO_A0_adcH, miniIO_A0_adcL, miniIO_A_relay, miniIO_A_inputs;
extern unsigned char			    miniIO_B1_adcH, miniIO_B1_adcL, miniIO_B0_adcH, miniIO_B0_adcL, miniIO_B_relay, miniIO_B_inputs;
extern unsigned char                old_tamper, old_link_port1, old_link_port2, old_link_port3;
extern CARD_TYPE					cardA_type, cardB_type;
extern struct io_descriptor		   *io;

// *****************************************************************************************************************************************************************
// Function:    convert(unsigned int num, int base)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: Function to format [vargs]
// Returns:     formatted string ptr
// *****************************************************************************************************************************************************************
static char *convert(unsigned int num, int base)
{
	static char		Representation[]= "0123456789ABCDEF";
	static char		buffer[50];
	char		   *ptr;
	
	ptr = &buffer[49];
	*ptr = '\0';
	
	do {
		*--ptr = Representation[num%base];
		num /= base;
	} while(num != 0);
	
	return ptr;
}

// *****************************************************************************************************************************************************************
// Function:    xprintf(char* format, ...) 
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: Function to format and write the debug/info to the standard stream IO
// Returns:     Nothing
// *****************************************************************************************************************************************************************
void xprintf(char* format, ...) 
{ 
	char			*traverse; 
	unsigned int     i;
	char			 temp;
	char            *s; 
	
	va_list arg; 
	va_start(arg, format); 
	
	for (traverse = format; *traverse; traverse++) 
	{ 
	    if (*traverse != '%') {
			while (!usart_async_is_tx_empty(&USART_1)) {}
			io_write((struct io_descriptor *const)io, (const uint8_t *const)traverse, 1);
			while (!usart_async_is_tx_empty(&USART_1)) {}
		} else {
			traverse++; 
			switch (*traverse)  { 
				case 'c' : i = va_arg(arg, int);
						temp = (char)i;
						while (!usart_async_is_tx_empty(&USART_1)) {}
						io_write((struct io_descriptor *const)io, (const uint8_t *const)&temp, 1);
						while (!usart_async_is_tx_empty(&USART_1)) {}
				break; 
						
				case 'd' : i = va_arg(arg,int);
						if(i < 0) 
						{ 
							i = -i;
							
							while (!usart_async_is_tx_empty(&USART_1)) {}
							io_write((struct io_descriptor *const)io, (const uint8_t *const)'-', 1);
							while (!usart_async_is_tx_empty(&USART_1)) {}
						} 
						
						s = convert(i, 10);
						
						while (!usart_async_is_tx_empty(&USART_1)) {}
						io_write((struct io_descriptor *const)io, (const uint8_t *const)s, strlen(s));
						while (!usart_async_is_tx_empty(&USART_1)) {}
				break; 
						
				case 'o': i = va_arg(arg,unsigned int);
						s = convert(i, 8);
						
						while (!usart_async_is_tx_empty(&USART_1)) {}
						io_write((struct io_descriptor *const)io, (const uint8_t *const)s, strlen(s));
						while (!usart_async_is_tx_empty(&USART_1)) {}
				break; 
						
				case 's': s = va_arg(arg,char *);
						while (!usart_async_is_tx_empty(&USART_1)) {}
						io_write((struct io_descriptor *const)io, (const uint8_t *const)s, strlen(s));
						while (!usart_async_is_tx_empty(&USART_1)) {}
				break; 
						
				case 'x': i = va_arg(arg,unsigned int);
						s = convert(i, 16);
						
						while (!usart_async_is_tx_empty(&USART_1)) {}
						io_write((struct io_descriptor *const)io, (const uint8_t *const)s, strlen(s));
						while (!usart_async_is_tx_empty(&USART_1)) {}
				break; 
			}	
		}
	} 
	
	va_end(arg); 
} 

// *****************************************************************************************************************************************************************
// Function:    read_boardvalues(void)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: read and access all main board hardware such as I2C and GPIO to collate a live snapshot of plugged-in cards and sensors
// Returns:     Nothing
// *****************************************************************************************************************************************************************
void read_boardvalues(void)
{
	int		err;
	
	if (read_hardware_index == 0) {
		hri_adc_write_INPUTCTRL_reg(ADC0, 0x1800 + 0);	/* Select AIN<0> - [Water 1] Analogue Input */
		delay_ms(2);
	
		/* Read the ADC value of the first water sensor. If bad result write error debug and set variable to 0xFF */
		err = adc_sync_read_channel(&ADC_0, 0, (uint8_t *const)&readdata_water1, 1);
		xprintf("err water 1 = %d\r\n", err);
	
		if (err != 1) {
			xprintf("Could NOT read from the water detector 1\r\n");
	
			readdata_water1 = 0;
		}
		
		read_hardware_index = 1;
	} else if (read_hardware_index == 1) {
		hri_adc_write_INPUTCTRL_reg(ADC0, 0x1800 + 2);	/* Select AIN<2> - [Water 2] Analogue Input */
		delay_ms(2);
	
		/* Read the ADC value of the second water sensor. If bad result write error debug and set variable to 0xFF */
		err = adc_sync_read_channel(&ADC_0, 0, (uint8_t *const)&readdata_water2, 1);
		xprintf("err water 2 = %d\r\n", err);
		
		if (err != 1) {
			xprintf("Could NOT read from the water detector 2\r\n");
			
			readdata_water2 = 0;
		}
		
		read_hardware_index = 2;
	} else if (read_hardware_index == 2) {
		hri_adc_write_INPUTCTRL_reg(ADC0, 0x1800 + 1);	/* Select AIN<1> - [HV Divider] Analogue Input */
		delay_ms(2);
		
		/* Read the ADC value of the [HV] divider. If bad result write error debug and set variable to 0x00 */
		err = adc_sync_read_channel(&ADC_0, 0, (uint8_t *const)&highvoltage, 1);
		xprintf("err HV = %d\r\n", err);
	
		if (err != 1) {
			xprintf("Could NOT read from the HV divider\r\n");
		
			highvoltage = 0;
		}
		
		read_hardware_index = 3;
	} else {
		/* Read the I2C for the dual temp & moisture IC. If bad result(s) write error debug and set variables to 0xFF */
		if ((err = I2C_getTEMPandMOISTURE(PB24_CARDB_I2C_SDA,  PB25_CARDB_I2C_CLK, &readdata_tempmoisture[0], &readdata_tempmoisture[1], &readdata_tempmoisture[2], &readdata_tempmoisture[3]) < 0)) {
			xprintf("Could NOT read from the moisture & temp sensor on CARDB:I2C [%i]\r\n", err);
		
			memset(&readdata_tempmoisture, 0xFF, 4);
		}
		
		read_hardware_index = 0;
	}

 xprintf("0 = %x\r\n", readdata_tempmoisture[0]);
 xprintf("1 = %x\r\n", readdata_tempmoisture[1]);
 xprintf("2 = %x\r\n", readdata_tempmoisture[2]);
 xprintf("3 = %x\r\n", readdata_tempmoisture[3]);

	/* Read the main simple GPIO inputs for determining the status of tamper, daughter cards */ 
	tamper = (gpio_get_pin_level(PB02_TAMP_OP)) ? 1 : 0;
	cardA_present = (gpio_get_pin_level(PC00_CARDA_PRESENT)) ? 0 : 1;
	cardB_present = (gpio_get_pin_level(PC01_CARDB_PRESENT)) ? 0 : 1;

	/* If tamper switch is closed and the enclosure lid is correct, then the LEDs do not illuminate. Else show the ETH and PWR LEDs according to the system state */
	if (tamper) {
		if (ring_broken == RING_BROKEN) {
			gpio_toggle_pin_level(PB03_LED_ETH);
			} else {
			gpio_set_pin_level(PB03_LED_ETH, (good_ethernet) ? 1 : 0);
		}
		
		gpio_set_pin_level(PB04_LED_PWR, 1);
	} else {
		gpio_set_pin_level(PB03_LED_ETH, 0);
		gpio_set_pin_level(PB04_LED_PWR, 0);
	}

	/* Read the status of the two LEFT and RIGHT main Ethernet ports, along with the T-junction Power & Ethernet adapter channel */
	link_port1 = (readKSZreg(SPI_KSZ8794_PORT1STATUS2) & 0x20) ? 1 : 0;
	link_port2 = (readKSZreg(SPI_KSZ8794_PORT2STATUS2) & 0x20) ? 1 : 0;
	link_port3 = (readKSZreg(SPI_KSZ8794_PORT3STATUS2) & 0x20) ? 1 : 0;

	/* Detect change of status for debug */
	if (tamper != old_tamper) {
		xprintf("Lid TAMPER [%s]\r\n", (!tamper) ? "GOOD" : "ACTIVE");
	}

	/* Detect change of status for debug */
	if (link_port1 != old_link_port1) {
		xprintf("Left Ethernet [%s]\r\n", (link_port1) ? "GOOD" : "LINKDOWN");
	}

	/* Detect change of status for debug */
	if (link_port2 != old_link_port2) {
		xprintf("Right Ethernet [%s]\r\n", (link_port2) ? "GOOD" : "LINKDOWN");
	}

	/* Detect change of status for debug */
	if (link_port3 != old_link_port3) {
		xprintf("Power & Ethernet <Comms Link> [%s]\r\n", (link_port3) ? "GOOD" : "DOWN");
	}

	/* Detect change of status of Slot [A] daughter-card */
	if ((!cardA_old) && (cardA_present)) {
		/* Read the I2C device address on the card to establish type of card */
	
		if (!(err = I2C_check_deviceID(PC27_CARDA_I2C_SDA, PC28_CARDA_I2C_CLK, 0x60))) {
			cardA_type = CARD_VIBRATEK;
		} else if (!(err = I2C_check_deviceID(PC27_CARDA_I2C_SDA, PC28_CARDA_I2C_CLK, 0x50))) {
			cardA_type = CARD_PE;
		} else if ((!(err = I2C_check_deviceID(PC27_CARDA_I2C_SDA, PC28_CARDA_I2C_CLK, 0x41))) && (!(err = I2C_check_deviceID(PC27_CARDA_I2C_SDA, PC28_CARDA_I2C_CLK, 0x68)))) {
			cardA_type = CARD_MINI_IO;
		} else {
			cardA_type = CARD_NOTFITTED;
		}

		if (err < 0) {
			xprintf("Could not read I2C bus on CARDA for I2C_check_deviceID() [%i]\r\n", err);
		}
		
		/* Actions to be taken for each different card type */
		switch (cardA_type) {
			case CARD_NOTFITTED:
			default:
			xprintf("CARDA inserted but not identified\r\n");
			break;
			
			case CARD_MAXI_IO:
			xprintf("CARDA is a MAXI IO\r\n");
			break;
			
			case CARD_MINI_IO:
			/* Set the configuration on the Mini-IO card */
			if ((err = I2C_configEXPANDER(PC27_CARDA_I2C_SDA,  PC28_CARDA_I2C_CLK) < 0)) {
				xprintf("Could NOT write to the Mini-IO Expander on CARDA:I2C [%i]\r\n", err);
			}

			xprintf("CARDA is a MINI IO\r\n");
			break;
			
			case CARD_VIBRAPOINT:
			xprintf("CARDA is a VIBRAPOINT\r\n");
			break;
			
			case CARD_VIBRATEK:
			/* Set the gain for the VibraTek card on the ADC reference voltage */
			if ((err = I2C_setGAIN(PC27_CARDA_I2C_SDA,  PC28_CARDA_I2C_CLK, ((CONFIG *)&settings_buffer)->gain_cardA) < 0)) {
				xprintf("Could NOT write to the DAC Gain controller on CARDA:I2C [%i]\r\n", err);
			}
			
			xprintf("CARDA is a VIBRATEK\r\n");
			break;
			
			case CARD_PE:
			/* Turn on the T-junction port 3 to the Power & Ethernet board attachment */
			writeKSZreg(SPI_KSZ8794_PORT3CONTROL2, 0x06);
			delay_us(50);
			
			if (!checkKSZreg(SPI_KSZ8794_PORT3CONTROL2, 0x06))	{ xprintf("[SPI_KSZ8794_PORT3CONTROL2] not correct\r\n"); }
				
			xprintf("CARDA is a Power & Ethernet Daughter-Card\r\n");
			break;
		}
	} else if (!cardA_present) {
		cardA_type = CARD_NOTFITTED;
		
		if (cardA_old) {
			/* Turn off the T-junction port 3 to the Power & Ethernet board attachment */
			writeKSZreg(SPI_KSZ8794_PORT3CONTROL2, 0x00);
			delay_us(50);
			
			if (!checkKSZreg(SPI_KSZ8794_PORT3CONTROL2, 0x00))	{ xprintf("[SPI_KSZ8794_PORT3CONTROL2] not correct\r\n"); }
				
			xprintf("CARDA has been REMOVED\r\n");
		}
	}

	/* Detect change of status of Slot [B] daughter-card */
	if ((!cardB_old) && (cardB_present)) {
		/* Read the I2C device address on the card to establish type of card */
		
		if (!(err = I2C_check_deviceID(PB24_CARDB_I2C_SDA, PB25_CARDB_I2C_CLK, 0x60))) {
			cardB_type = CARD_VIBRATEK;
		} else if ((!(err = I2C_check_deviceID(PB24_CARDB_I2C_SDA, PB25_CARDB_I2C_CLK, 0x41))) && (!(err = I2C_check_deviceID(PB24_CARDB_I2C_SDA, PB25_CARDB_I2C_CLK, 0x68)))) {
			cardB_type = CARD_MINI_IO;		
		} else {
			cardB_type = CARD_NOTFITTED;
		}
		
		if (err < 0) {
			xprintf("Could not read I2C bus on CARDB for I2C_check_deviceID() [%i]\r\n", err);
		}
		
		/* Actions to be taken for each different card type */
		switch (cardB_type) {
			case CARD_NOTFITTED:
			default:
			xprintf("CARDB inserted but not identified\r\n");
			break;
			
			case CARD_MAXI_IO:
			xprintf("CARDB is a MAXI IO\r\n");
			break;
			
			case CARD_MINI_IO:
			/* Set the configuration on the Mini-IO card */
			if ((err = I2C_configEXPANDER(PB24_CARDB_I2C_SDA,  PB25_CARDB_I2C_CLK) < 0)) {
				xprintf("Could NOT write to the Mini-IO Expander on CARDB:I2C [%i]\r\n", err);
			}
			
			xprintf("CARDB is a MINI IO\r\n");
			break;
			
			case CARD_VIBRAPOINT:
			xprintf("CARDB is a VIBRAPOINT\r\n");
			break;
			
			case CARD_VIBRATEK:
			/* Set the gain for the VibraTek card on the ADC reference voltage */
			if ((err = I2C_setGAIN(PB24_CARDB_I2C_SDA,  PB25_CARDB_I2C_CLK, ((CONFIG *)&settings_buffer)->gain_cardB)) < 0) {
				xprintf("Could NOT write to the DAC Gain controller on CARDB:I2C [%i]\r\n", err);
			}
			
			xprintf("CARDB is a VIBRATEK\r\n");
			break;
		}
	} else if (!cardB_present) {
		cardB_type = CARD_NOTFITTED;
		
		if (cardA_old) {
			xprintf("CARDB has been REMOVED\r\n");
		}
	}

	/* Update the old live states so that next time round, we can detect any changes since this time */
	cardA_old = cardA_present;
	cardB_old = cardB_present;
	old_tamper = tamper;
	old_link_port1 = link_port1;
	old_link_port2 = link_port2;
	old_link_port3 = link_port3;
}