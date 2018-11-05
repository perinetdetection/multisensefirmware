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

/* external functions list */
extern void writeKSZreg(uint16_t, unsigned char);
extern bool checkKSZreg(uint16_t, unsigned char);

/* global variables list */
static  unsigned char	wbuf[99], mode, first, hysterysis, port_val;

// *****************************************************************************************************************************************************************
// Function:    ring_init(unsigned char basestation, unsigned char *mac_raw)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: Called by the main code application to setup and initialize the BPDU ring topology management
// Returns:     Nothing (NULL)
// *****************************************************************************************************************************************************************
void ring_init(unsigned char basestation, unsigned char *mac_raw)
{
	int				loop;

	/* set flags to indicate first time boot-up */
	first = 1;
	port_val = 0;

	/* "basestation" parameter determines if this MultiSense unit is a base-station or simple pass-through... */
	if (basestation) {
		xprintf("ETHERNET: Set as a base-station to manage Ethernet topology...\r\n");

		/* Enable BPDU tags */
		writeKSZreg(SPI_KSZ8794_ACCESS_DATA8, 0x00);
		writeKSZreg(SPI_KSZ8794_ACCESS_DATA7, 0x00);
		writeKSZreg(SPI_KSZ8794_ACCESS_DATA6, 0x30);
		writeKSZreg(SPI_KSZ8794_ACCESS_DATA5, 0x01);
		writeKSZreg(SPI_KSZ8794_ACCESS_DATA4, 0x80);
		writeKSZreg(SPI_KSZ8794_ACCESS_DATA3, 0xC2);
		writeKSZreg(SPI_KSZ8794_ACCESS_DATA2, 0x00);
		writeKSZreg(SPI_KSZ8794_ACCESS_DATA1, 0x00);
		writeKSZreg(SPI_KSZ8794_ACCESS_DATA0, 0x00);
		writeKSZreg(SPI_KSZ8794_ACCESS_CONTROL0, 0x00);
		writeKSZreg(SPI_KSZ8794_ACCESS_CONTROL1, 0x00);
	
		/* Set the local context "mode" to indicate this is in fact a base-station */
		mode = 1;
	}
	else {
		xprintf("ETHERNET: normal none-managed pass-through node...\r\n");
		
		/* Set the local context "mode" to indicate this is in fact a pass-though un-configured node */
		mode = 0;
	}
	
	hysterysis = 0;

	/* Bridge DST MAC */
	wbuf[0] = 0x01;
	wbuf[1] = 0x80;
	wbuf[2] = 0xC2;
	wbuf[3] = 0x00;
	wbuf[4] = 0x00;
	wbuf[5] = 0x00;

	/* Bridge SRC MAC */
	wbuf[6] = 0x01;
	wbuf[7] = 0x80;
	wbuf[8] = 0xC2;
	wbuf[9] = 0x00;
	wbuf[10] = 0x00;
	wbuf[11] = 0x01;

	/* BPDU TAG field. Forced forwarding, filtering, priority, send to Port 0 ONLY */
	*((unsigned short int *)&wbuf[12]) = 0x01E0;
	
	wbuf[14] = 0x00;
	wbuf[15] = 0x07;
	wbuf[16] = 0x00;
	wbuf[17] = 0x00;
	wbuf[18] = 0x00;
	wbuf[19] = 0x01;

	/* BPDU Length/Type */
	wbuf[20] = 0x00;
	wbuf[21] = 0x27;
	wbuf[22] = 0x42;
	wbuf[23] = 0x42;
    wbuf[24] = 0x03;

	/* BPDU Header */
	wbuf[25] = 0x00;
	wbuf[26] = 0x00;
	wbuf[27] = 0x00;
	wbuf[28] = 0x00;

	/* BPDU data payload */
	wbuf[29] = mac_raw[0];
	wbuf[30] = mac_raw[1];
	wbuf[31] = mac_raw[2];
	wbuf[32] = mac_raw[3];
	wbuf[33] = mac_raw[4];
	wbuf[34] = mac_raw[5];

	/* BPDU Padding ZEROs */
	for (loop = 35; loop < 68; loop++) {
		wbuf[loop] = 0x00;
	}
	
	wbuf[68] = 0x02;																											/* Tail TAG to inform SWITCH to forward to <Port 2> */

	xprintf("ETHERNET: Completed RING management init...\r\n");

	/* If we are a base-station, then send out the first BPDU frame now upon start-up */
	if (basestation) {		
		writeKSZreg(SPI_KSZ8794_GLOBAL10, 0x46);																				/* When [BS] switched selected, we are a base-station */
		writeKSZreg(SPI_KSZ8794_PORT2CONTROL2, 0x04);
		mac_async_write(&ETHERNET_MAC_0, (uint8_t *)&wbuf[0], 69);
		writeKSZreg(SPI_KSZ8794_PORT2CONTROL2, 0x00);
		writeKSZreg(SPI_KSZ8794_GLOBAL10, 0x44);
		
		xprintf("ETHERNET: Sent first BPDU...\r\n");
	}
}

// *****************************************************************************************************************************************************************
// Function:    ring_check(unsigned char bpdu_arrived, unsigned char *storm_state, unsigned char *ringbreak)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: Called by the main loop execution and checks every time to see if if storms and BPDUs arrive. This manages the storm until it subsides
// Returns:     Nothing (NULL)
// *****************************************************************************************************************************************************************
unsigned char ring_check(unsigned char bpdu_arrived, unsigned char *storm_state, unsigned char *ringbreak)
{
	unsigned char	sendrelearn;

	/* Bound-checks and assertions */
	if (!storm_state) {
		xprintf("ETHERNET: (!storm_state) ERROR\r\n");
		return 0;
	}

	if (!ringbreak) {
		xprintf("ETHERNET: (!ringbreak) ERROR\r\n");
		return 0;
	}

	sendrelearn = 0;

	/* Check which mode we are running, base-station or pass-though? */
	if (mode) {
		/* If no BPDU arrived as yet, this indicates a ring break potentially */
		if ((!bpdu_arrived) && (!(*storm_state))) {
			if ((first) || ((!(*ringbreak))) && (hysterysis == 2)) {
				/* Enable Port 2 Management port back to normal */
				writeKSZreg(SPI_KSZ8794_PORT2CONTROL2, 0x06);
				port_val = 0x06;
				
				/* Clear the learning tables in the SWITCH */
				writeKSZreg(SPI_KSZ8794_GLOBAL0, 0x2C);
				delay_us(250);

				/* Set flags to indicate to the main-loop that we have a broken network topology */
				sendrelearn = 1;
				*ringbreak = 1;

				xprintf("ETHERNET: [turning Ethernet <B> ON - RING BROKEN]...\r\n");
			} else {
				hysterysis++;
			}
		}
		else if (!(*storm_state)) {
			/* If BPDU has arrived, this indicates a healed network topology with loop intact */
			if ((first) || (*ringbreak)) {
				/* Disable Port 2 Management port but crucially keep transmitting/receiving BPDUs */
				writeKSZreg(SPI_KSZ8794_PORT2CONTROL2, 0x00);
				port_val = 0;
				
				/* Clear the learning tables in the SWITCH */
				writeKSZreg(SPI_KSZ8794_GLOBAL0, 0x2C);
				delay_us(250);

				/* Set flags to indicate to the main-loop that we have a healed and fully looped network topology */
				sendrelearn = 1;
				*ringbreak = 0;
				hysterysis = 0;

				xprintf("ETHERNET: [turning Ethernet <B> OFF - RING INTACT]...\r\n");
				
				while (mac_async_read_len(&ETHERNET_MAC_0) > 0) {
						mac_async_read(&ETHERNET_MAC_0, (uint8_t *)&uip_buf[0], sizeof(uip_buf));
				}
			}
		}
	}

	/* Now check to see what the situation with broadcast storm entails */
	if (*storm_state) {
		xprintf("ETHERNET: [storm subsided]...\r\n");
		*storm_state = 0;
		
		/* Clear the learning tables in the SWITCH */
		writeKSZreg(SPI_KSZ8794_GLOBAL0, 0x2C);

		if (mode) {
			port_val = 0;
			delay_us(250);
		
			/* Set flags to indicate to the main-loop that we have a healed and fully looped network topology */
			sendrelearn = 1;
			*ringbreak = 0;

			xprintf("ETHERNET: [turning Ethernet <B> OFF - RING INTACT]...\r\n");
				
			while (mac_async_read_len(&ETHERNET_MAC_0) > 0) {
				mac_async_read(&ETHERNET_MAC_0, (uint8_t *)&uip_buf[0], sizeof(uip_buf));
			}
			
			hysterysis = 0;
		}
	}

    /* However, if we have no such storm conditions, and we are a base-station, then send the next BPDU out */
	if (mode) {
		writeKSZreg(SPI_KSZ8794_PORT2CONTROL2, (0x04 | port_val));
		writeKSZreg(SPI_KSZ8794_GLOBAL10, 0x46);
		mac_async_write(&ETHERNET_MAC_0, (uint8_t *)&wbuf[0], 69);
		writeKSZreg(SPI_KSZ8794_PORT2CONTROL2, port_val);
		writeKSZreg(SPI_KSZ8794_GLOBAL10, 0x44);
	}

	first = 0;
	return sendrelearn;
}