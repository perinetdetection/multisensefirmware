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

/* external timer list */
extern struct timer_task			TIMER_0_task1;
extern void							usb_device_cb_state_c(void);

/* external global variables list */
extern unsigned int					tick_counter;
extern unsigned char				readenvironment, arp_check, sentA, sentB, reboot_actioned, refresh_gain;
extern unsigned char	            readdata_tempmoisture[4], readdata_water1, readdata_water2, highvoltage;
extern unsigned char				send_relearn_udp;
extern unsigned char				ring_timer;
extern unsigned char		 		stormstate;
extern unsigned char		 		settings_buffer[], ring, read_hardware_index;
extern unsigned int				    card_sampleindex, looprate, loopcount;
extern unsigned char				tamper, cardA_present, cardB_present, cardA_old, cardB_old, link_port1, link_port2, link_port3, ring_broken, miniA_chan, miniB_chan;
extern unsigned char	        	miniIO_A1_adcH, miniIO_A1_adcL, miniIO_A0_adcH, miniIO_A0_adcL, miniIO_A_relay, miniIO_A_inputs;
extern unsigned char			    miniIO_B1_adcH, miniIO_B1_adcL, miniIO_B0_adcH, miniIO_B0_adcL, miniIO_B_relay, miniIO_B_inputs;
extern unsigned char                old_tamper, old_link_port1, old_link_port2, old_link_port3, temp_failure_flag;
extern CARD_TYPE					cardA_type, cardB_type;
extern struct spi_xfer				p_xfer;
extern struct uip_eth_addr			macaddress;
extern uint8_t						mac_raw[];
extern uip_ipaddr_t					ipaddr, netmask, gwaddr, broadcast;
extern struct uip_udp_conn		   *main_socket, *cardA_socket, *cardB_socket, *ring_socket;
extern uint8_t						iv[];
extern uint8_t						aes_key[];
extern struct io_descriptor		   *io;

/* external functions list */
extern int							EEprom_settings(unsigned char *, unsigned int, unsigned char);
extern bool							checkKSZreg(uint16_t, unsigned char);
extern unsigned char				readKSZreg(uint16_t);
extern void							writeKSZreg(uint16_t, unsigned char);
extern void							TIMER_0_task1_cb(const struct timer_task *const);

// *****************************************************************************************************************************************************************
// Function:    timer_setup(void)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: initializes and starts the main 10 times per second global timer
// Returns:     Nothing
// *****************************************************************************************************************************************************************
void timer_setup(void)
{
	/* Setup and start the main 1/10th of a second timer expiration callback routine */
	TIMER_0_task1.interval = 100;
	TIMER_0_task1.cb       = TIMER_0_task1_cb;
	TIMER_0_task1.mode     = TIMER_TASK_REPEAT;

	/* Start the repeatable timer feature */
	timer_add_task(&TIMER_0, &TIMER_0_task1);
	timer_start(&TIMER_0);
}

// *****************************************************************************************************************************************************************
// Function:    ADC_init(void)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: initializes and starts the two internal ADC features for reading the water detection mechanism
// Returns:     Nothing
// *****************************************************************************************************************************************************************
void ADC_init(void)
{
	/* Setup the two internal water detection analogue inputs BOTH on "ADC_0" channels and also the HIGH-VOLTAGE ADC [0|1|2]*/
	adc_sync_enable_channel(&ADC_0, 0);
}

// *****************************************************************************************************************************************************************
// Function:    comms_init(void)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: setup the SPI and the USB
// Returns:     Nothing
// *****************************************************************************************************************************************************************
void comms_init(void)
{
	/* Global initialization of main USB mechanism */
	if (usb_d_enable() < 0) {
		xprintf("USB init/enable ERROR!!!!!!!\r\n");
	} else {
		usb_init();
		xprintf("usb_init()\r\n");
		
		cdc_device_acm_init();
		xprintf("cdc_device_acm_init()\r\n");
		
		/* Register the read callback function */
		cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usb_device_cb_state_c);
		xprintf("cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usb_device_cb_state_c)\r\n");
	}
}

// *****************************************************************************************************************************************************************
// Function:    crypto_init(void)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: initialize the AES encryption-engine in the CPU core
// Returns:     Nothing
// *****************************************************************************************************************************************************************
void crypto_init(void)
{
	/* Initialize and start the crypt hardware block. Insert and program the private AES key for this device-node also */
	aes_sync_enable(&CRYPTOGRAPHY_0);
	aes_sync_set_encrypt_key(&CRYPTOGRAPHY_0, aes_key, AES_KEY_128);
}

// *****************************************************************************************************************************************************************
// Function:    switch_init(void)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: reset and start the Ethernet SWITCH IC
// Returns:     Nothing
// *****************************************************************************************************************************************************************
void switch_init(void)
{
	/* Go through the reset sequence of the GPIO for the Micrel Ethernet-SWITCH */
	gpio_set_pin_level(PB00_KSZ_RESET, 1);
	delay_ms(10);
	gpio_set_pin_level(PB00_KSZ_RESET, 0);
	delay_ms(100);
	gpio_set_pin_level(PB00_KSZ_RESET, 1);
}

// *****************************************************************************************************************************************************************
// Function:    switch_configure(void)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: check all default register settings in the Ethernet SWITCH and also change relevant settings to suit the MultiSense operation
// Returns:     Nothing
// *****************************************************************************************************************************************************************
void switch_configure(void)
{
	xprintf("Configuring and checking the SPI KSZ8794 Ethernet SWITCH registers...\r\n");
	
	/* Write set-up commands to the KSZ Ethernet SWITCH via SPI bus */
	if (!checkKSZreg(SPI_KSZ8794_FAMILY_ID, 0x87))		{ xprintf("[SPI_KSZ8794_FAMILY_ID] not correct\r\n"); }
		
	/* Stop the Ethernet SWITCH operation */
	writeKSZreg(SPI_KSZ8794_START, 0x00);
	writeKSZreg(SPI_KSZ8794_GLOBAL2, 0xB0);
	writeKSZreg(SPI_KSZ8794_GLOBAL6, 0x80);
	delay_us(50);
	
	if (!checkKSZreg(SPI_KSZ8794_START, 0x60))			{ xprintf("[SPI_KSZ8794_START first] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_GLOBAL0, 0x0C))		{ xprintf("[SPI_KSZ8794_GLOBAL0] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_GLOBAL1, 0x04))		{ xprintf("[SPI_KSZ8794_GLOBAL1] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_GLOBAL2, 0xB0))		{ xprintf("[SPI_KSZ8794_GLOBAL2] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_GLOBAL3, 0x00))		{ xprintf("[SPI_KSZ8794_GLOBAL3] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_GLOBAL4, 0x00))		{ xprintf("[SPI_KSZ8794_GLOBAL4] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_GLOBAL5, 0x4A))		{ xprintf("[SPI_KSZ8794_GLOBAL5] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_GLOBAL6, 0x00))		{ xprintf("[SPI_KSZ8794_GLOBAL6] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_GLOBAL9, 0x00))		{ xprintf("[SPI_KSZ8794_GLOBAL9] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_GLOBAL10, 0x44))		{ xprintf("[SPI_KSZ8794_GLOBAL10] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PDMC1, 0x00))			{ xprintf("[SPI_KSZ8794_PDMC1] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PDMC2, 0x50))			{ xprintf("[SPI_KSZ8794_PDMC2] not correct\r\n"); }
	
	writeKSZreg(SPI_KSZ8794_PORT1CONTROL0, 0x80);
	delay_us(50);
	
	if (!checkKSZreg(SPI_KSZ8794_PORT1CONTROL0, 0x80))	{ xprintf("[SPI_KSZ8794_PORT1CONTROL0] not correct\r\n"); }
	
	writeKSZreg(SPI_KSZ8794_PORT2CONTROL0, 0x80);
	delay_us(50);
	
	if (!checkKSZreg(SPI_KSZ8794_PORT2CONTROL0, 0x80))	{ xprintf("[SPI_KSZ8794_PORT2CONTROL0] not correct\r\n"); }
	
	writeKSZreg(SPI_KSZ8794_PORT3CONTROL0, 0x80);
	delay_us(50);
	
	if (!checkKSZreg(SPI_KSZ8794_PORT3CONTROL0, 0x80))	{ xprintf("[SPI_KSZ8794_PORT3CONTROL0] not correct\r\n"); }
	
	writeKSZreg(SPI_KSZ8794_PORT4CONTROL0, 0x80);
	delay_us(50);
	
	if (!checkKSZreg(SPI_KSZ8794_PORT4CONTROL0, 0x80))	{ xprintf("[SPI_KSZ8794_PORT4CONTROL0] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT1CONTROL1, 0x1F))	{ xprintf("[SPI_KSZ8794_PORT1CONTROL1] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT2CONTROL1, 0x1F))	{ xprintf("[SPI_KSZ8794_PORT2CONTROL1] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT3CONTROL1, 0x1F))	{ xprintf("[SPI_KSZ8794_PORT3CONTROL1] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT4CONTROL1, 0x1F))	{ xprintf("[SPI_KSZ8794_PORT4CONTROL1] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT1CONTROL2, 0x06))	{ xprintf("[SPI_KSZ8794_PORT1CONTROL2] not correct\r\n"); }
	
	if (((CONFIG *)&settings_buffer)->loop_basestation) {	
		writeKSZreg(SPI_KSZ8794_PORT2CONTROL2, 0x00);	
		if (!checkKSZreg(SPI_KSZ8794_PORT2CONTROL2, 0x00))	{ xprintf("[SPI_KSZ8794_PORT2CONTROL2] not correct\r\n"); }
	} else {
		writeKSZreg(SPI_KSZ8794_PORT2CONTROL2, 0x06);	
		if (!checkKSZreg(SPI_KSZ8794_PORT2CONTROL2, 0x06))	{ xprintf("[SPI_KSZ8794_PORT2CONTROL2] not correct\r\n"); }
	}
	
	/* For now, turn off the T-junction port 3 to the Power & Ethernet board attachment */
	writeKSZreg(SPI_KSZ8794_PORT3CONTROL2, 0x00);
	delay_us(50);
	
	if (!checkKSZreg(SPI_KSZ8794_PORT3CONTROL2, 0x00))	{ xprintf("[SPI_KSZ8794_PORT3CONTROL2] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT4CONTROL2, 0x06))	{ xprintf("[SPI_KSZ8794_PORT4CONTROL2] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT1CONTROL3, 0x00))	{ xprintf("[SPI_KSZ8794_PORT1CONTROL3] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT2CONTROL3, 0x00))	{ xprintf("[SPI_KSZ8794_PORT2CONTROL3] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT3CONTROL3, 0x00))	{ xprintf("[SPI_KSZ8794_PORT3CONTROL3] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT4CONTROL3, 0x00))	{ xprintf("[SPI_KSZ8794_PORT4CONTROL3] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT1CONTROL4, 0x01))	{ xprintf("[SPI_KSZ8794_PORT1CONTROL4] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT2CONTROL4, 0x01))	{ xprintf("[SPI_KSZ8794_PORT2CONTROL4] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT3CONTROL4, 0x01))	{ xprintf("[SPI_KSZ8794_PORT3CONTROL4] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT4CONTROL4, 0x01))	{ xprintf("[SPI_KSZ8794_PORT4CONTROL4] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT1CONTROL5, 0x00))	{ xprintf("[SPI_KSZ8794_PORT1CONTROL5] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT2CONTROL5, 0x00))	{ xprintf("[SPI_KSZ8794_PORT2CONTROL5] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT3CONTROL5, 0x00))	{ xprintf("[SPI_KSZ8794_PORT3CONTROL5] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT4CONTROL5, 0x00))	{ xprintf("[SPI_KSZ8794_PORT4CONTROL5] not correct\r\n"); }
		
	writeKSZreg(SPI_KSZ8794_PORT4CONTROL6, 0x20);
	delay_us(50);
	
	if (!checkKSZreg(SPI_KSZ8794_PORT4CONTROL6, 0x20))	{ xprintf("[SPI_KSZ8794_PORT4CONTROL6] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT1CONTROL7, 0x3F))	{ xprintf("[SPI_KSZ8794_PORT1CONTROL7] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT2CONTROL7, 0x3F))	{ xprintf("[SPI_KSZ8794_PORT2CONTROL7] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT3CONTROL7, 0x3F))	{ xprintf("[SPI_KSZ8794_PORT3CONTROL7] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT1CONTROL8, 0x00))	{ xprintf("[SPI_KSZ8794_PORT1CONTROL8] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT2CONTROL8, 0x00))	{ xprintf("[SPI_KSZ8794_PORT2CONTROL8] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT3CONTROL8, 0x00))	{ xprintf("[SPI_KSZ8794_PORT3CONTROL8] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT1LINKMD, 0x00))	{ xprintf("[SPI_KSZ8794_PORT1LINKMD] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT2LINKMD, 0x00))	{ xprintf("[SPI_KSZ8794_PORT2LINKMD] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT3LINKMD, 0x00))	{ xprintf("[SPI_KSZ8794_PORT3LINKMD] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT1CONTROL9, 0x5F))	{ xprintf("[SPI_KSZ8794_PORT1CONTROL9] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT2CONTROL9, 0x5F))	{ xprintf("[SPI_KSZ8794_PORT2CONTROL9] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT3CONTROL9, 0x5F))	{ xprintf("[SPI_KSZ8794_PORT3CONTROL9] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT1CONTROL10, 0x00)) { xprintf("[SPI_KSZ8794_PORT1CONTROL10] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT2CONTROL10, 0x00)) { xprintf("[SPI_KSZ8794_PORT2CONTROL10] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT3CONTROL10, 0x00)) { xprintf("[SPI_KSZ8794_PORT3CONTROL10] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT2CONTROL20, 0x62)) { xprintf("[SPI_KSZ8794_PORT2CONTROL20] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT1CONTROL12, 0x80)) { xprintf("[SPI_KSZ8794_PORT1CONTROL12] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT2CONTROL12, 0x80)) { xprintf("[SPI_KSZ8794_PORT2CONTROL12] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT3CONTROL12, 0x80)) { xprintf("[SPI_KSZ8794_PORT3CONTROL12] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT4CONTROL12, 0x80)) { xprintf("[SPI_KSZ8794_PORT4CONTROL12] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT1CONTROL13, 0x00)) { xprintf("[SPI_KSZ8794_PORT1CONTROL13] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT2CONTROL13, 0x00)) { xprintf("[SPI_KSZ8794_PORT2CONTROL13] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT3CONTROL13, 0x00)) { xprintf("[SPI_KSZ8794_PORT3CONTROL13] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT4CONTROL13, 0x00)) { xprintf("[SPI_KSZ8794_PORT4CONTROL13] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT1CONTROL14, 0x88)) { xprintf("[SPI_KSZ8794_PORT1CONTROL14] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT2CONTROL14, 0x88)) { xprintf("[SPI_KSZ8794_PORT2CONTROL14] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT3CONTROL14, 0x88)) { xprintf("[SPI_KSZ8794_PORT3CONTROL14] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT4CONTROL14, 0x88)) { xprintf("[SPI_KSZ8794_PORT4CONTROL14] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT1CONTROL15, 0x84)) { xprintf("[SPI_KSZ8794_PORT1CONTROL15] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT2CONTROL15, 0x84)) { xprintf("[SPI_KSZ8794_PORT2CONTROL15] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT3CONTROL15, 0x84)) { xprintf("[SPI_KSZ8794_PORT3CONTROL15] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT4CONTROL15, 0x84)) { xprintf("[SPI_KSZ8794_PORT4CONTROL15] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT1CONTROL16, 0x82)) { xprintf("[SPI_KSZ8794_PORT1CONTROL16] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT2CONTROL16, 0x82)) { xprintf("[SPI_KSZ8794_PORT2CONTROL16] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT3CONTROL16, 0x82)) { xprintf("[SPI_KSZ8794_PORT3CONTROL16] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT4CONTROL16, 0x82)) { xprintf("[SPI_KSZ8794_PORT4CONTROL16] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT1CONTROL17, 0x81)) { xprintf("[SPI_KSZ8794_PORT1CONTROL17] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT2CONTROL17, 0x81)) { xprintf("[SPI_KSZ8794_PORT2CONTROL17] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT3CONTROL17, 0x81)) { xprintf("[SPI_KSZ8794_PORT3CONTROL17] not correct\r\n"); }
	if (!checkKSZreg(SPI_KSZ8794_PORT4CONTROL17, 0x81)) { xprintf("[SPI_KSZ8794_PORT4CONTROL17] not correct\r\n"); }
		
	/* Start the Ethernet SWITCH operation */
	writeKSZreg(SPI_KSZ8794_START, 0x01);
	delay_us(50);
	
	/* Check it has re-started and running */
	if (!checkKSZreg(SPI_KSZ8794_START, 0x61))			{ xprintf("[SPI_KSZ8794_START final] not correct\r\n"); }
	/* Completed set-up commands to the KSZ Ethernet SWITCH via SPI bus */
}
	
// *****************************************************************************************************************************************************************
// Function:    address_configure(void)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: from the ID value, set the global IP address, IP scheme and MAC address
// Returns:     Nothing
// *****************************************************************************************************************************************************************
void address_configure(void)
{	
	if (((CONFIG *)&settings_buffer)->ID == 0x00000000) {
		xprintf("MAC address and IP scheme not initialized as there is no [ID] set...\r\n");
		return;
	}
	
	/* Set the first 4-octets of the "MAC_PREFIX" for all the MultiSense boards */
	mac_raw[0] = macaddress.addr[0] = MAC_ADDR_00;
	mac_raw[1] = macaddress.addr[1] = MAC_ADDR_01;
	mac_raw[2] = macaddress.addr[2] = (uint8_t)(((((CONFIG *)&settings_buffer)->ID) >> 24 & 0x00FF));
	mac_raw[3] = macaddress.addr[3] = (uint8_t)(((((CONFIG *)&settings_buffer)->ID) >> 16) & 0x00FF);
	mac_raw[4] = macaddress.addr[4] = (uint8_t)(((((CONFIG *)&settings_buffer)->ID) >> 8) & 0x00FF);
    mac_raw[5] = macaddress.addr[5] = (uint8_t)((((CONFIG *)&settings_buffer)->ID) & 0x00FF);
	
	/* Update the uIP stack with these values */
	uip_setethaddr(macaddress);
		
	/* Start the main internal on-chip Ethernet MAC for the frames */
	//mac_async_set_filter_ex(&ETHERNET_MAC_0, mac_raw);
	mac_async_enable(&ETHERNET_MAC_0);
	
	uip_ipaddr(&ipaddr, 192, 168, (uint8_t)((((CONFIG *)&settings_buffer)->ID) / 250), 1 + (uint8_t)((((CONFIG *)&settings_buffer)->ID) % 250));

	/* Update the uIP stack with these values */
	uip_ipaddr(&gwaddr, 192, 168, 0, 1);									// IP address of CPU Server on installation network is always "192.168.0.1". This address can NOT be held by any Multi-Sense node
	uip_ipaddr(&netmask, 255, 255, 0, 0);
	uip_ipaddr(&broadcast, 255, 255, 255, 255);
	uip_sethostaddr(&ipaddr);
	uip_setnetmask(&netmask);
	uip_setdraddr(&gwaddr);

	xprintf("MultiSense [init MAC/IP values] %x:%x:%x:%x:%x:%x %d.%d.%d.%d\r\n", mac_raw[0], mac_raw[1], mac_raw[2], mac_raw[3], mac_raw[4], mac_raw[5], uip_ipaddr1(ipaddr), uip_ipaddr2(ipaddr), uip_ipaddr3(ipaddr), uip_ipaddr4(ipaddr));
	
	/* Create and bind the main UDP socket for the MultiSense board */
	main_socket = uip_udp_new(&gwaddr, htons(MAIN_UDPSOCKET));
	if (main_socket != NULL) {
		uip_udp_bind(main_socket, htons(MAIN_UDPSOCKET));
		xprintf("Main UDP socket created...\r\n");
	} else {
		xprintf("Main UDP socket ERROR!!!!!!!\r\n");
	}
	
	/* Create and bind the main UDP socket for the daughter-card on Slot [A] */
	cardA_socket = uip_udp_new(&gwaddr, htons(CARDA_UDPSOCKET));
	if (cardA_socket != NULL) {
		uip_udp_bind(cardA_socket, htons(CARDA_UDPSOCKET));
		xprintf("Card [A] UDP socket created...\r\n");
	} else {
		xprintf("Card [A] UDP socket ERROR!!!!!!!\r\n");
	}
	
	/* Create and bind the main UDP socket for the daughter-card on Slot [B] */
	cardB_socket = uip_udp_new(&gwaddr, htons(CARDB_UDPSOCKET));
	if (cardB_socket != NULL) {
		uip_udp_bind(cardB_socket, htons(CARDB_UDPSOCKET));
		xprintf("Card [B] UDP socket created...\r\n");
	} else {
		xprintf("Card [B] UDP socket ERROR!!!!!!!\r\n");
	}
	
	/* Create and bind the main UDP socket for the MAC address table FLUSH broadcast when controlling the network loop-topology */
	ring_socket = uip_udp_new(&broadcast, htons(RING_MANAGEMENT_SOCKET));
	if (ring_socket != NULL) {
		uip_udp_bind(ring_socket, htons(RING_MANAGEMENT_SOCKET));
		xprintf("Ring Management UDP socket created...\r\n");
	} else {
		xprintf("Ring Management UDP socket ERROR!!!!!!!\r\n");
	}
}
	
// *****************************************************************************************************************************************************************
// Function:    gpio_init(void)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: define, set and configure the [GPIO] characteristics of the SAM CPU IC
// Returns:     Nothing
// *****************************************************************************************************************************************************************
void gpio_init(void)
{
	/* Ensure all GPIO MultiSense pins are not accidentally attached to unintended hardware blocks in the SAM CPU */
	gpio_set_pin_function(PB00_KSZ_RESET, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_function(PB02_TAMP_OP, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_function(PB03_LED_ETH, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_function(PB04_LED_PWR, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_function(PB05_SPInCS_CARDA, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_function(PB06_SPInCS_CARDB, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_function(PB07_SPInCS_KSZ8974, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_function(PC00_CARDA_PRESENT, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_function(PC01_CARDB_PRESENT, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_function(PB24_CARDB_I2C_SDA, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_function(PB25_CARDB_I2C_CLK, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_function(PC27_CARDA_I2C_SDA, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_function(PC28_CARDA_I2C_CLK, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_function(PB12_SPI_MOSI, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_function(PB13_SPI_MISO, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_function(PB15_SPI_CLK, GPIO_PIN_FUNCTION_OFF);

	/* Set all GPIO MultiSense pins to their respective INPUT or OUTPUT directions */
	gpio_set_pin_direction(PB00_KSZ_RESET, GPIO_DIRECTION_OUT);
	gpio_set_pin_direction(PB02_TAMP_OP, GPIO_DIRECTION_IN);
	gpio_set_pin_direction(PB03_LED_ETH, GPIO_DIRECTION_OUT);
	gpio_set_pin_direction(PB04_LED_PWR, GPIO_DIRECTION_OUT);
	gpio_set_pin_direction(PB05_SPInCS_CARDA, GPIO_DIRECTION_OUT);
	gpio_set_pin_direction(PB06_SPInCS_CARDB, GPIO_DIRECTION_OUT);
	gpio_set_pin_direction(PB07_SPInCS_KSZ8974, GPIO_DIRECTION_OUT);
	gpio_set_pin_direction(PC00_CARDA_PRESENT, GPIO_DIRECTION_IN);
	gpio_set_pin_direction(PC01_CARDB_PRESENT, GPIO_DIRECTION_IN);
	gpio_set_pin_direction(PB24_CARDB_I2C_SDA, GPIO_DIRECTION_IN);
	gpio_set_pin_direction(PB25_CARDB_I2C_CLK, GPIO_DIRECTION_OUT);
	gpio_set_pin_direction(PC27_CARDA_I2C_SDA, GPIO_DIRECTION_IN);
	gpio_set_pin_direction(PC28_CARDA_I2C_CLK, GPIO_DIRECTION_OUT);
	gpio_set_pin_direction(PB12_SPI_MOSI, GPIO_DIRECTION_OUT);
	gpio_set_pin_direction(PB13_SPI_MISO, GPIO_DIRECTION_IN);
	gpio_set_pin_direction(PB15_SPI_CLK, GPIO_DIRECTION_OUT);

	/* Set all GPIO MultiSense pins to their respective internal pull-up or pull-down configurations. Most of the GPIO has external PCB resistor pull-ups or pull-downs */
	gpio_set_pin_pull_mode(PB00_KSZ_RESET, GPIO_PULL_OFF);
	gpio_set_pin_pull_mode(PB02_TAMP_OP, GPIO_PULL_OFF);
	gpio_set_pin_pull_mode(PB03_LED_ETH, GPIO_PULL_OFF);
	gpio_set_pin_pull_mode(PB04_LED_PWR, GPIO_PULL_OFF);
	gpio_set_pin_pull_mode(PB05_SPInCS_CARDA, GPIO_PULL_OFF);
	gpio_set_pin_pull_mode(PB06_SPInCS_CARDB, GPIO_PULL_OFF);
	gpio_set_pin_pull_mode(PB07_SPInCS_KSZ8974, GPIO_PULL_OFF);
	gpio_set_pin_pull_mode(PC00_CARDA_PRESENT, GPIO_PULL_UP);
	gpio_set_pin_pull_mode(PC01_CARDB_PRESENT, GPIO_PULL_UP);
	gpio_set_pin_pull_mode(PB24_CARDB_I2C_SDA, GPIO_PULL_OFF);
	gpio_set_pin_pull_mode(PB25_CARDB_I2C_CLK, GPIO_PULL_OFF);
	gpio_set_pin_pull_mode(PC27_CARDA_I2C_SDA, GPIO_PULL_OFF);
	gpio_set_pin_pull_mode(PC28_CARDA_I2C_CLK, GPIO_PULL_OFF);
	gpio_set_pin_pull_mode(PB13_SPI_MISO, GPIO_PULL_OFF);
	
	/* Set the initial levels of the GPIO output pins including the LEDs and the SPI slave select lines */
	gpio_set_pin_level(PB03_LED_ETH, 0);
	gpio_set_pin_level(PB04_LED_PWR, 0);
	gpio_set_pin_level(PB05_SPInCS_CARDA, 1);
	gpio_set_pin_level(PB06_SPInCS_CARDB, 1);
	gpio_set_pin_level(PB07_SPInCS_KSZ8974, 1);
	gpio_set_pin_level(PB25_CARDB_I2C_CLK, 0);
	gpio_set_pin_level(PC28_CARDA_I2C_CLK, 0);
	gpio_set_pin_level(PB12_SPI_MOSI, 1);
	gpio_set_pin_level(PB15_SPI_CLK, 1);
	gpio_set_pin_level(PB00_KSZ_RESET, 1);
}
	
// *****************************************************************************************************************************************************************
// Function:    watchdog_init(void)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: setup, initialize and start the watchdog code timer
// Returns:     Nothing
// *****************************************************************************************************************************************************************
void watchdog_init(void)
{
	/* Set the main watchdog to 4 seconds timeout */
	wdt_set_timeout_period(&WDT_0, 100, 25);
	
	/* Turn it on */
	wdt_enable(&WDT_0);
}

// *****************************************************************************************************************************************************************
// Function:    var_init(void)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: reset all global variables
// Returns:     Nothing
// *****************************************************************************************************************************************************************
void var_init(void)
{
	/* Set all the main global variables to zero to initialize the states before nay further code executes that relies on the initialization process at the start */
	tick_counter = 0;
	readenvironment = 0;
	arp_check = 0;
	ring_timer = 0;
	ring = 0;

	sentA = 0;
	sentB = 0;
	send_relearn_udp = 0;
	refresh_gain = 0;
	
	cardA_old = 0;
	cardB_old = 0;
	card_sampleindex = 0;
	stormstate = 0;
	link_port1 = 0;
	link_port2 = 0;
	link_port3 = 0;
	old_tamper = 0;
	old_link_port1 = 0;
	old_link_port2 = 0;
	old_link_port3 = 0;
	readdata_water1 = 0;
	readdata_water2 = 0;
	highvoltage = 0;
	read_hardware_index = 0;
	looprate = 0;
	loopcount = 0;
	miniA_chan = 0;
	miniB_chan = 0;
	miniIO_A1_adcH = 0;
	miniIO_A1_adcL = 0;
	miniIO_A0_adcH = 0;
	miniIO_A0_adcL = 0;
	miniIO_A_relay = 0;
	miniIO_A_inputs = 0;
	miniIO_B1_adcH = 0;
	miniIO_B1_adcL = 0;
	miniIO_B0_adcH = 0;
	miniIO_B0_adcL = 0;
	miniIO_B_relay = 0;
	miniIO_B_inputs = 0;
	
	temp_failure_flag = 0;
	
	/* Set the enumerated type variables such as the daughter card types and the network loop ring-topology management state */
	cardA_type = CARD_NOTFITTED;
	cardB_type = CARD_NOTFITTED;
	ring_broken = RING_NOTCONFIGURED;
	
	/* Also, read out the non-volatile EEprom configuration data */
	if (EEprom_settings(settings_buffer, SETTING_STRUCTURE_SIZE, 0) != ERR_NONE) {
		xprintf("MultiSense [configuration ERROR READING]\r\n");
	}
	
	/* Check if the configuration data is invalid and needs defaulting */
	if ((((CONFIG *)&settings_buffer)->pattern1 != CONFIG_IDENTIFIER1) || (((CONFIG *)&settings_buffer)->pattern2 != CONFIG_IDENTIFIER2)) {
		xprintf("MultiSense [configuration INVALID]\r\n");
		
		/* Re-write default values ready for write-back */
		((CONFIG *)&settings_buffer)->pattern1 = CONFIG_IDENTIFIER1;
		((CONFIG *)&settings_buffer)->pattern2 = CONFIG_IDENTIFIER2;
		strncpy((char *__restrict)(((CONFIG *)&settings_buffer)->name), "MULTISENSE NEEDS CONFIGURING...", sizeof(((CONFIG *)&settings_buffer)->name));
		((CONFIG *)&settings_buffer)->ID = 0x00000000;
		((CONFIG *)&settings_buffer)->gain_cardA = 0xFF;
		((CONFIG *)&settings_buffer)->gain_cardB = 0xFF;
		((CONFIG *)&settings_buffer)->loop_basestation = 0;
		
		xprintf("MultiSense [configuration FORMATTED]\r\n");

		/* Write these values back to the EEprom storage area */
		if (EEprom_settings(settings_buffer, SETTING_STRUCTURE_SIZE, 1) != ERR_NONE) {
			xprintf("MultiSense [configuration ERROR WRITING]\r\n");
		} else {
			xprintf("MultiSense [configuration WRITTEN]\r\n");
		}
	}
}