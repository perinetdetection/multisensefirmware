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
#include "stdio.h"
#include "usb_protocol.h"
#include "hpl_wdt.h"
#include "hpl_dma.h"
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

//#define USE_ENCRYPTION

#define	MULTISENSE_FIRMWARE_MAJOR	0
#define	MULTISENSE_FIRMWARE_MINOR	1

#define DMA_CHANNEL_0				0
#define SEQ_LENGTH					10
#define ETH_BUF			    		((struct uip_eth_hdr *)&uip_buf[0])
#define CIPHERTEXT   			    uip_appdata
#define PLAINTEXT   			    plain_buffer

extern int							I2C_configMONITOREDINPUTS(uint8_t,  uint8_t, unsigned char);
extern int							I2C_readMONITOREDINPUT(uint8_t,  uint8_t, unsigned char *, unsigned char *);
extern int							I2C_configEXPANDER(uint8_t,  uint8_t);
extern int							I2C_setEXPANDER(uint8_t,  uint8_t, unsigned char);
extern int							I2C_readEXPANDER(uint8_t,  uint8_t, unsigned char *);
extern int							I2C_setGAIN(uint8_t sda,  uint8_t, unsigned char);
extern int							I2C_check_deviceID(uint8_t, uint8_t, unsigned char);
extern void							ring_init(unsigned char, unsigned char *);
extern unsigned char				ring_check(unsigned char, unsigned char *, unsigned char *);
extern int							EEprom_settings(unsigned char *, unsigned int, unsigned char);
extern bool							checkKSZreg(uint16_t, unsigned char);
extern unsigned char				readKSZreg(uint16_t);
extern void							writeKSZreg(uint16_t, unsigned char);
extern void							application_appcall(void);
extern void							application_init(void);
extern void							tcpip_output(void);
extern void							uip_log(char *);
extern void							timer_setup(void);
extern void							comms_init(void);
extern void							crypto_init(void);
extern void							switch_init(void);
extern void							switch_configure(void);
extern void							address_configure(void);
extern void							gpio_init(void);
extern void							watchdog_init(void);
extern void							var_init(void);
extern void							ADC_init(void);
extern void							read_boardvalues(void);

struct timer_task					TIMER_0_task1;

unsigned int						tick_counter;
unsigned char						readenvironment, arp_check, ip_periodic_check, sentA, sentB, refresh_gain;
unsigned char						send_relearn_udp;
unsigned char						ring_timer;
unsigned char		 				stormstate;
unsigned char		 				ring, read_hardware_index;

unsigned char						plain_buffer[1024];
unsigned char						cardAch0_samplebuffer[SAMPLE_BUFFER_SIZE];
unsigned char						cardAch1_samplebuffer[SAMPLE_BUFFER_SIZE];
unsigned char						cardAch2_samplebuffer;
unsigned char						cardAch3_samplebuffer;
unsigned char						cardBch0_samplebuffer[SAMPLE_BUFFER_SIZE];
unsigned char						cardBch1_samplebuffer[SAMPLE_BUFFER_SIZE];
unsigned char						cardBch2_samplebuffer;
unsigned char						cardBch3_samplebuffer;
unsigned int						card_sampleindex, looprate, loopcount;

unsigned char						settings_buffer[SETTING_STRUCTURE_SIZE], readdata_tempmoisture[4], readdata_water1, readdata_water2, highvoltage, command_dataw[2], command_datar[2];
unsigned char						tamper, cardA_present, cardB_present, cardA_old, cardB_old, good_ethernet, link_port1, link_port2, link_port3, ring_broken, miniA_chan, miniB_chan;
unsigned char						miniIO_A1_adcH, miniIO_A1_adcL, miniIO_A0_adcH, miniIO_A0_adcL, miniIO_A_relay, miniIO_A_inputs;
unsigned char						miniIO_B1_adcH, miniIO_B1_adcL, miniIO_B0_adcH, miniIO_B0_adcL, miniIO_B_relay, miniIO_B_inputs;
unsigned char						old_tamper, old_link_port1, old_link_port2, old_link_port3;
CARD_TYPE							cardA_type, cardB_type;
struct uip_eth_addr					macaddress;
uint8_t								mac_raw[6];
uip_ipaddr_t						ipaddr, netmask, gwaddr, broadcast;
struct uip_udp_conn				   *main_socket, *cardA_socket, *cardB_socket, *ring_socket;
uint8_t								iv[16];

/* USART IO descriptor*/
struct io_descriptor			   *io;

/* DMA update sequence by writing the respective DSEQCTRL bit */
volatile uint32_t					inputctrl_buff[1] = {0x0003};

// All Multi-Sense nodes have this private-secret [KEY]
uint8_t								aes_key[16] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};

// *****************************************************************************************************************************************************************
// Function:    bash_spi_transfer(unsigned char *tx, unsigned char *rx, int size)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: rolls round the push-pull MOSI/MISO of the SPI bus for [size] bytes of 8-bits, MSB
// Returns:     Nothing
// *****************************************************************************************************************************************************************
static void inline bash_spi_transfer(unsigned char *tx, unsigned char *rx, int size)
{
	int register count, loop;
	
	for (count = 0; count < size; count++) {
		rx[count] = 0;
		
		for (loop = 8; loop; loop--) {
			gpio_set_pin_level(PB12_SPI_MOSI, (tx[count] & (1 << (loop - 1))) ? 1 : 0);
			gpio_set_pin_level(PB15_SPI_CLK, 0);
			gpio_set_pin_level(PB15_SPI_CLK, 0);
			gpio_set_pin_level(PB15_SPI_CLK, 0);
			gpio_set_pin_level(PB15_SPI_CLK, 1);
			
			rx[count] |= (gpio_get_pin_level(PB13_SPI_MISO) << (loop - 1));
		}
	}
}

// *****************************************************************************************************************************************************************
// Function:    TIMER_0_task1_cb(const struct timer_task *const timer_task)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: When a 100ms timer expires, this callback function is executed. this signifies the 1/10th of a second timer tick
// Returns:     Nothing
// *****************************************************************************************************************************************************************
void TIMER_0_task1_cb(const struct timer_task *const timer_task)
{
	/* Increment and if necessary wrap-around the timer counter */
	tick_counter++;

	/* Every 5/10th of a second, set the readenvironment flag to instigate the board live readings to be take down in the main-loop */
	if (!(tick_counter % 5)) {
		readenvironment = 1;
		
		/* Kick the watchdog time-out facility */
	    wdt_feed(&WDT_0);
	}

	/* Every 1/10th of a second, poll theuIP stack as part of the pre-requisite requirement for the stack */
	ip_periodic_check = 1;

	/* Every 2.5 seconds, call the ARP-timer check in the uIP stack */
	if (!(tick_counter % 25)) {
		arp_check = 1;
	}

	/* Every 5/10th of a second, set the flag to check the loop BPDU ring-management function */
	if (!(tick_counter % 5)) {
		ring_timer = 1;
	}
	
	/* Every second, make a note of the amount of main loop iterations per second */
	if (!(tick_counter % 10)) {
		looprate = loopcount;
		loopcount = 0;
	}

	/* If the "good_ethernet" counter was set, then this decrements it in 1/10ths of a second, such that after 25 seconds, if not packet activity, the [LED_ETH] turns off */
	if (good_ethernet) {
		good_ethernet--;
	}
}

// *****************************************************************************************************************************************************************
// Function:    application_udp_appcall(void)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: Main application thread for the UDP channels
// Returns:     Nothing (NULL)
// *****************************************************************************************************************************************************************
void application_udp_appcall(void)
{
	u16_t	len;

	/* If the uIP stack signals that we have a new UDPmpacket RX, then execute next block of code */
	if (uip_newdata()) {
		/* Keep the "good_ethernet" counter topped up */
		good_ethernet = 255;

		/* Found out how much payload data we have in the UDP packet */
		len = uip_datalen();

		/* If this UDP packet is for the main-board then ... */
		if (uip_conn->lport == htons(MAIN_UDPSOCKET)) {
#ifdef USE_ENCRYPTION
			memmove(iv, aes_key, 16);
			aes_sync_cbc_crypt(&CRYPTOGRAPHY_0, AES_DECRYPT, CIPHERTEXT, PLAINTEXT, len, &iv[0]);					// Use AES 128-bit encryption algorithm
#else
			memmove(PLAINTEXT, CIPHERTEXT, len);																	// If no encryption compiled, then just copy over plain-text from [UDP] buffer
#endif
			if ((len == 39) && (PLAINTEXT[0] == ID_IDENT_1) && (PLAINTEXT[1] == ID_IDENT_2) && (PLAINTEXT[2] == ID_IDENT_3) && (PLAINTEXT[3] == ID_IDENT_4)) {
				/* If this was an [ID_SET] packet... */
				memmove(((CONFIG *)&settings_buffer)->name, &PLAINTEXT[4], 33);

				/* Move all new settings and ID values into the configuration EEprom overlay */
				((CONFIG *)&settings_buffer)->gain_cardA = PLAINTEXT[37];
				((CONFIG *)&settings_buffer)->gain_cardB = PLAINTEXT[38];

				/* Then write-back the overlay back into NV ram store */
				if (EEprom_settings(settings_buffer, SETTING_STRUCTURE_SIZE, 1) != ERR_NONE) {
					xprintf("MultiSense [configuration ERROR READING]\r\n");
				} else {
					xprintf("MultiSense [UPDATED <ID> configuration WRITTEN]\r\n");
				}

				/* Signal to the main-loop, that we must update the ADC gains if there are VibraTek daughter-cards installed */
				refresh_gain = 1;
				xprintf("Global settings packet received...\r\n");	
			}
		} else if (uip_conn->lport == htons(CARDA_UDPSOCKET)) {
#ifdef USE_ENCRYPTION
			memmove(iv, aes_key, 16);
			aes_sync_cbc_crypt(&CRYPTOGRAPHY_0, AES_DECRYPT, CIPHERTEXT, PLAINTEXT, len, &iv[0]);					// Use AES 128-bit encryption algorithm
#else
			memmove(PLAINTEXT, CIPHERTEXT, len);																	// If no encryption compiled, then just copy over plain-text from [UDP] buffer
#endif						
			/* Actions to be taken for each different card type */
			switch (cardA_type) {
				case CARD_NOTFITTED:
				default:
				
				break;
				
				case CARD_MAXI_IO:
				
				break;
				
				case CARD_MINI_IO:
				if (len == 1) {
					miniIO_A_relay = PLAINTEXT[0];
					xprintf("CardA Mini-IO packet received...\r\n");
				}
				break;
				
				case CARD_VIBRAPOINT:
				
				break;
				
				case CARD_VIBRATEK:
			
				break;
				
				case CARD_PE:
				
				break;
			}
		} else if (uip_conn->lport == htons(CARDB_UDPSOCKET)) {
#ifdef USE_ENCRYPTION
			memmove(iv, aes_key, 16);
			aes_sync_cbc_crypt(&CRYPTOGRAPHY_0, AES_DECRYPT, CIPHERTEXT, PLAINTEXT, len, &iv[0]);					// Use AES 128-bit encryption algorithm
#else
			memmove(PLAINTEXT, CIPHERTEXT, len);																	// If no encryption compiled, then just copy over plain-text from [UDP] buffer
#endif
			/* Actions to be taken for each different card type */
			switch (cardB_type) {
				case CARD_NOTFITTED:
				default:
				
				break;
				
				case CARD_MAXI_IO:
				
				break;
				
				case CARD_MINI_IO:
				if (len == 1) {
					miniIO_B_relay = PLAINTEXT[0];
					xprintf("CardB Mini-IO packet received...\r\n");
				}
				break;
				
				case CARD_VIBRAPOINT:
				
				break;
				
				case CARD_VIBRATEK:
				
				break;
				
				case CARD_PE:
				
				break;
			}
		} else if (uip_conn->lport == htons(RING_MANAGEMENT_SOCKET)) {
			/* If we are a pass-through device and not configured as a base-station, then be ready to receive a FLUSH MAC table packet */
			if (!(((CONFIG *)&settings_buffer)->loop_basestation)) {
				/* UDP FLUSH packet arrived, so start the SPI register write process of flushing the MAc tables in the Ethernet SWITCH IC */
				writeKSZreg(SPI_KSZ8794_PORT1CONTROL2, 0x07);
				writeKSZreg(SPI_KSZ8794_PORT2CONTROL2, 0x07);

				/* Clear the learning tables in the SWITCH */
				writeKSZreg(SPI_KSZ8794_GLOBAL0, 0x2D);
				delay_us(250);

				/* Once cleared, turn off learnign state for ports 1 and 2 */
				writeKSZreg(SPI_KSZ8794_PORT1CONTROL2, 0x06);
				writeKSZreg(SPI_KSZ8794_PORT2CONTROL2, 0x06);

				xprintf("Broadcast Table [FLUSH] UDP packet arrived...\r\n");
			}
		}
	}

	/* If we are just polling to see if UDP packets need sending out then... */
	if ((uip_rexmit()) || (uip_poll())) {
		if (uip_conn->lport == htons(MAIN_UDPSOCKET)) {
			if (readenvironment == 2) {
				/* If there is a collection of board data ready to be sent out then... */
				PLAINTEXT[0] = (uint8_t)(((((CONFIG *)&settings_buffer)->ID) >> 24) & 0x00FF);
				PLAINTEXT[1] = (uint8_t)(((((CONFIG *)&settings_buffer)->ID) >> 16) & 0x00FF);
				PLAINTEXT[2] = (uint8_t)(((((CONFIG *)&settings_buffer)->ID) >> 8) & 0x00FF);
				PLAINTEXT[3] = (uint8_t)((((CONFIG *)&settings_buffer)->ID) & 0x00FF);
				memmove(&PLAINTEXT[4], ((CONFIG *)&settings_buffer)->name, 33);
				PLAINTEXT[37] = ((CONFIG *)&settings_buffer)->gain_cardA;
				PLAINTEXT[38] = ((CONFIG *)&settings_buffer)->gain_cardB;
				PLAINTEXT[39] = ((CONFIG *)&settings_buffer)->loop_basestation;
				PLAINTEXT[40] = readdata_water1;
				PLAINTEXT[41] = readdata_water2;
				PLAINTEXT[42] = highvoltage;
				PLAINTEXT[43] = readdata_tempmoisture[0];
				PLAINTEXT[44] = readdata_tempmoisture[1];
				PLAINTEXT[45] = readdata_tempmoisture[2];
				PLAINTEXT[46] = readdata_tempmoisture[3];
				PLAINTEXT[47] = tamper;
				PLAINTEXT[48] = link_port1;
				PLAINTEXT[49] = link_port2;
				PLAINTEXT[50] = link_port3;
				PLAINTEXT[51] = cardA_type;
				PLAINTEXT[52] = cardB_type;
				PLAINTEXT[53] = ring_broken;
				PLAINTEXT[54] = MULTISENSE_FIRMWARE_MAJOR;
				PLAINTEXT[55] = MULTISENSE_FIRMWARE_MINOR;
				PLAINTEXT[56] = ID_IDENT_1;
				PLAINTEXT[57] = ID_IDENT_2;
				PLAINTEXT[58] = ID_IDENT_3;
				PLAINTEXT[59] = ID_IDENT_4;
				PLAINTEXT[60] = 0;

				/* Once the PLAINTEXT data buffer has been filled out with the live information...*/
#ifdef USE_ENCRYPTION
				memmove(iv, aes_key, 16);
				aes_sync_cbc_crypt(&CRYPTOGRAPHY_0, AES_ENCRYPT, PLAINTEXT, CIPHERTEXT, 61, &iv[0]);	    // Use AES 128-bit encryption algorithm
#else
				memmove(CIPHERTEXT, PLAINTEXT, 61);														// If no encryption compiled, then just copy over plain-text into [UDP] buffer
#endif
				/* Send the actual main board UDP packet to the CPU-Server */
				uip_udp_send(61);

				/* Clear the flag that indicated we had live data to send */
				readenvironment = 0;
				xprintf("Hardware packet send...\r\n");
			}
		} else if (uip_conn->lport == htons(CARDA_UDPSOCKET)) {
			/* Actions to be taken for each different card type */
			switch (cardA_type) {
				case CARD_NOTFITTED:
				default:
					
				break;
					
				case CARD_MAXI_IO:
					
				break;
					
				case CARD_MINI_IO:
				/* If there is a collection of board data ready to be sent out then... */
				PLAINTEXT[0] = (uint8_t)miniIO_A0_adcH;
				PLAINTEXT[1] = (uint8_t)miniIO_A0_adcL;
				PLAINTEXT[2] = (uint8_t)miniIO_A1_adcH;
				PLAINTEXT[3] = (uint8_t)miniIO_A1_adcL;
				PLAINTEXT[4] = (uint8_t)miniIO_A_inputs;

				/* Once the PLAINTEXT data buffer has been filled out with the live information...*/
#ifdef USE_ENCRYPTION
				memmove(iv, aes_key, 16);
				aes_sync_cbc_crypt(&CRYPTOGRAPHY_0, AES_ENCRYPT, PLAINTEXT, CIPHERTEXT, 5, &iv[0]);	   // Use AES 128-bit encryption algorithm
#else
				memmove(CIPHERTEXT, PLAINTEXT, 5);													// If no encryption compiled, then just copy over plain-text into [UDP] buffer
#endif
				/* Send the actual main board UDP packet to the CPU-Server */
				uip_udp_send(3);
				xprintf("CardA Mini-IO packet send...\r\n");	
				break;
					
				case CARD_VIBRAPOINT:
					
				break;
					
				case CARD_VIBRATEK:
				if ((!sentA) && (card_sampleindex == SAMPLE_BUFFER_SIZE)) {
					/* If there is a collection of Card A sampled data ready to be sent out then... */
					memmove(&PLAINTEXT[0], cardAch0_samplebuffer, SAMPLE_BUFFER_SIZE);
					memmove(&PLAINTEXT[SAMPLE_BUFFER_SIZE], cardAch1_samplebuffer, SAMPLE_BUFFER_SIZE);
					PLAINTEXT[(SAMPLE_BUFFER_SIZE * 2) + 0] = cardAch2_samplebuffer;
					PLAINTEXT[(SAMPLE_BUFFER_SIZE * 2) + 1] = cardAch3_samplebuffer;
					PLAINTEXT[(SAMPLE_BUFFER_SIZE * 2) + 2] = ID_IDENT_1;
					PLAINTEXT[(SAMPLE_BUFFER_SIZE * 2) + 3] = ID_IDENT_2;
					PLAINTEXT[(SAMPLE_BUFFER_SIZE * 2) + 4] = ID_IDENT_3;
					PLAINTEXT[(SAMPLE_BUFFER_SIZE * 2) + 5] = ID_IDENT_4;

					/* Once the PLAINTEXT data buffer has been filled out with the sampled data...*/
#ifdef USE_ENCRYPTION
					memmove(iv, aes_key, 16);
					aes_sync_cbc_crypt(&CRYPTOGRAPHY_0, AES_ENCRYPT, PLAINTEXT, CIPHERTEXT, (SAMPLE_BUFFER_SIZE * 2) + 6, &iv[0]);	// Use AES 128-bit encryption algorithm
#else
					memmove(CIPHERTEXT, PLAINTEXT, (SAMPLE_BUFFER_SIZE * 2) + 6);													// If no encryption compiled, then just copy over plain-text into [UDP] buffer
#endif
					/* Send the Card A UDP packet to the CPU-Server */
					uip_udp_send((SAMPLE_BUFFER_SIZE * 2) + 6);

					/* Ensure that we do not reset "card_sampleindex" back to zero until we know that the other Card B has been sent also */
					if (sentB) {
						sentB = 0;
						card_sampleindex = 0;
						} else {
						sentA = 1;
					}
						
					xprintf("CardA VibraTek packet send...\r\n");
				}
				break;
					
				case CARD_PE:
					
				break;
			}
		} else if (uip_conn->lport == htons(CARDB_UDPSOCKET)) {
			/* Actions to be taken for each different card type */
			switch (cardB_type) {
				case CARD_NOTFITTED:
				default:
				
				break;
				
				case CARD_MAXI_IO:
				
				break;
				
				case CARD_MINI_IO:
				/* If there is a collection of board data ready to be sent out then... */
				PLAINTEXT[0] = (uint8_t)miniIO_B0_adcH;
				PLAINTEXT[1] = (uint8_t)miniIO_B0_adcL;
				PLAINTEXT[2] = (uint8_t)miniIO_B1_adcH;
				PLAINTEXT[3] = (uint8_t)miniIO_B1_adcL;
				PLAINTEXT[4] = (uint8_t)miniIO_B_inputs;

				/* Once the PLAINTEXT data buffer has been filled out with the live information...*/
#ifdef USE_ENCRYPTION
				memmove(iv, aes_key, 16);
				aes_sync_cbc_crypt(&CRYPTOGRAPHY_0, AES_ENCRYPT, PLAINTEXT, CIPHERTEXT, 5, &iv[0]);	   // Use AES 128-bit encryption algorithm
#else
				memmove(CIPHERTEXT, PLAINTEXT, 3);												   	// If no encryption compiled, then just copy over plain-text into [UDP] buffer
#endif
				/* Send the actual main board UDP packet to the CPU-Server */
				uip_udp_send(5);
				xprintf("CardB Mini-IO packet send...\r\n");

				break;
				
				case CARD_VIBRAPOINT:
				
				break;
				
				case CARD_VIBRATEK:
				if ((!sentB) && (card_sampleindex == SAMPLE_BUFFER_SIZE)) {
					/* If there is a collection of Card B sampled data ready to be sent out then... */
					memmove(&PLAINTEXT[0], cardBch0_samplebuffer, SAMPLE_BUFFER_SIZE);
					memmove(&PLAINTEXT[SAMPLE_BUFFER_SIZE], cardBch1_samplebuffer, SAMPLE_BUFFER_SIZE);
					PLAINTEXT[(SAMPLE_BUFFER_SIZE * 2) + 0] = cardBch2_samplebuffer;
					PLAINTEXT[(SAMPLE_BUFFER_SIZE * 2) + 1] = cardBch3_samplebuffer;
					PLAINTEXT[(SAMPLE_BUFFER_SIZE * 2) + 2] = ID_IDENT_1;
					PLAINTEXT[(SAMPLE_BUFFER_SIZE * 2) + 3] = ID_IDENT_2;
					PLAINTEXT[(SAMPLE_BUFFER_SIZE * 2) + 4] = ID_IDENT_3;
					PLAINTEXT[(SAMPLE_BUFFER_SIZE * 2) + 5] = ID_IDENT_4;

					/* Once the PLAINTEXT data buffer has been filled out with the sampled data...*/
#ifdef USE_ENCRYPTION
					memmove(iv, aes_key, 16);
					aes_sync_cbc_crypt(&CRYPTOGRAPHY_0, AES_ENCRYPT, PLAINTEXT, CIPHERTEXT, (SAMPLE_BUFFER_SIZE * 2) + 6, &iv[0]);	// Use AES 128-bit encryption algorithm
#else
					memmove(CIPHERTEXT, PLAINTEXT, (SAMPLE_BUFFER_SIZE * 2) + 6);													// If no encryption compiled, then just copy over plain-text into [UDP] buffer
#endif
					/* Send the Card B UDP packet to the CPU-Server */
					uip_udp_send((SAMPLE_BUFFER_SIZE * 2) + 6);

					/* Ensure that we do not reset "card_sampleindex" back to zero until we know that the other Card A has been sent also */
					if (sentA) {
						sentA = 0;
						card_sampleindex = 0;
						} else {
						sentB = 1;
					}
				
					xprintf("CardB VibraTek packet send...\r\n");
				}
				break;
				
				case CARD_PE:
				
				break;
			}
		} else if (uip_conn->lport == htons(RING_MANAGEMENT_SOCKET)) {
			/* If this poll event is for the BPDU ring network topology channel then... */
			if ((send_relearn_udp) && (((CONFIG *)&settings_buffer)->loop_basestation)) {
				/* If we are configured as a base-station and a FLUSH MAC packet is ready to be sent out then... */
				PLAINTEXT[0] = 0;
				/* Once the PLAINTEXT data buffer has been filled out with a dummy byte then...*/
				memmove(CIPHERTEXT, PLAINTEXT, 1);

				/* Send the FLUSH MAC table UDP packet to all other pass-through MultiSense devices on the network */
				uip_udp_send(1);
																																// Send a padded none-encrypted zero as dummy pay-load for table flush packet
				send_relearn_udp = 0;
				xprintf("Topology update packet send...\r\n");
			}
		}
	}
}

// *****************************************************************************************************************************************************************
// Function:    main(void)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: Main start-point of whole application code
// Returns:     Nothing (NULL)
// *****************************************************************************************************************************************************************
int main_loop(void)
{
	uint8_t			ch;
	unsigned char	key;
	int				err, loop, etherloop, eth_check, key_check;
	unsigned char	bpdu_arrived, broadcast_storm;

	/* Zero-off and initialize the local variables here */
	bpdu_arrived = 0;
	broadcast_storm = 0;
	eth_check = 0;
	key_check = 0;
	command_dataw[1] = 0x00;
	command_dataw[2] = 0x00;

	/* This is the forever loop that is the "main-loop" of the whole firmware in MultiSense */
	while (1) {
		/* reset the flag that indicates that a BPDU has arrived */
		bpdu_arrived = 0;
		loopcount++;

		/* Only every 1000 loops do we check for incoming network frames */
		if (eth_check == 1000) {
			/* Be prepared to burst read up to 5 frames this check */
			for (etherloop = 0; etherloop < 5; etherloop++) {
				/* What is the length of the next incoming Ethernet frame? */
				if (mac_async_read_len(&ETHERNET_MAC_0) > 0) {
					/* If none-zero, then there is another packet to be read, and go ahead and read it from the MAC */
					uip_len = mac_async_read(&ETHERNET_MAC_0, (uint8_t *)&uip_buf[0], sizeof(uip_buf));
				} else {
					/* If no more, then escape for-loop check */
					break;
				}

				/* If incoming frame is a BPDU with the correct identifiers etc then... */
				if ((uip_len == 68) && (uip_buf[0] == 0x01) && (uip_buf[1] == 0x80) && (uip_buf[2] == 0xC2) && (uip_buf[3] == 0x00) && (uip_buf[4] == 0x00) && (uip_buf[5] == 0x00) && (uip_buf[6] == 0x01) &&
				(uip_buf[7] == 0x80) && (uip_buf[8] == 0xC2) && (uip_buf[9] == 0x00) && (uip_buf[10] == 0x00) && (uip_buf[11] == 0x01) && (*((unsigned short int *)&(uip_buf[12])) == 0x01E0) &&
				(uip_buf[29] == mac_raw[0]) && (uip_buf[30] == mac_raw[1]) && (uip_buf[31] == mac_raw[2]) && (uip_buf[32] == mac_raw[3]) && (uip_buf[33] == mac_raw[4]) && (uip_buf[34] == mac_raw[5])) {
					/* We have received a BPDU frame */
					bpdu_arrived = 1;
				} else {
					/* Normal frame arrived */
					if (ETH_BUF->type == htons(UIP_ETHTYPE_IP)) {
						/* Frame is an Ethernet type */
						uip_arp_ipin();
						uip_input();
						/* Process incoming frame in the uIP stack */

						if (uip_len > 0) {
							/* If there is resultant output data to be sent then send it now after an ARP look-up first */
							uip_arp_out();
							mac_async_write(&ETHERNET_MAC_0, (uint8_t *)&uip_buf[0], uip_len);
						}
					} else if (ETH_BUF->type == htons (UIP_ETHTYPE_ARP)) {
						/* If the incoming frame is an ARP packet */
						uip_arp_arpin();

						/* If any data resulting from the received ARP packet needs to be sent out, then do it now */
						if (uip_len > 0) {
							mac_async_write(&ETHERNET_MAC_0, (uint8_t *)&uip_buf[0], uip_len);
						}
					}
				}
			}

			/* If there is more than 5 consecutive incoming packets read out, then we declare that we are in a broadcast storm state */
			if (etherloop == 5) {
				if (!broadcast_storm) {
					xprintf("ETHERNET: [broadcast storm] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");

					/* Disable Port 4 Management port but crucially keep transmitting/receiving BPDUs */
					writeKSZreg(SPI_KSZ8794_PORT4CONTROL2, 0x00);
					stormstate = 1;
					broadcast_storm = 1;
				}
			}
			else if (!stormstate) {
				/* Otherwise, if less than 5 frames received at once, then we are not in a storm state */
				broadcast_storm = 0;
			}

			/* Every 10 times per second, poll theuIP stack */
			if (ip_periodic_check) {
				ip_periodic_check = 0;

				/* Check all sockets */
				for (loop = 0; loop < UIP_UDP_CONNS; loop++) {
					uip_periodic(loop);

					/* Any resultant outbound packets, send out now */
					if (uip_len > 0) {
						uip_arp_out();
						mac_async_write(&ETHERNET_MAC_0, (uint8_t *)&uip_buf[0], uip_len);
					}
				}

				/* Check all UDP connection sockets */
				for (loop = 0; loop < UIP_UDP_CONNS; loop++) {
					uip_udp_periodic(loop);

					/* Any resultant outbound packets, send out now */
					if (uip_len > 0) {
						uip_arp_out();
						mac_async_write(&ETHERNET_MAC_0, (uint8_t *)&uip_buf[0], uip_len);
					}
				}
			}

			/* Every 2.5 seconds check the ARP state-machine */
			if (arp_check) {
				arp_check = 0;

				uip_arp_timer();
			}

			eth_check = 0;
		} else {
			eth_check++;
		}

		/* If the configuration settings have bee changed, then update the gain ADC settings on the VibraTek cards if they are installed... */
		if (refresh_gain) {
			if (cardA_type == CARD_VIBRATEK) {
				/* VibraTek Card A installed... */
				xprintf("Write ADC gain settings for VibraTek CardA\r\n");

				/* Update the gain ADC value via I2C write operation */
				if ((err = I2C_setGAIN(PC27_CARDA_I2C_SDA,  PC28_CARDA_I2C_CLK, ((CONFIG *)&settings_buffer)->gain_cardA)) < 0) {
					xprintf("Could NOT write to the DAC Gain controller on CARDA:I2C [%i]\r\n", err);
				}
			}

			if (cardB_type == CARD_VIBRATEK) {
				/* VibraTek Card B installed... */
				xprintf("Write ADC gain settings for VibraTek CardB\r\n");

				/* Update the gain ADC value via I2C write operation */
				if (( err = I2C_setGAIN(PB24_CARDB_I2C_SDA,  PB25_CARDB_I2C_CLK, ((CONFIG *)&settings_buffer)->gain_cardB)) < 0) {
					xprintf("Could NOT write to the DAC Gain controller on CARDB:I2C [%i]\r\n", err);
				}
			}

			/* Clear the flag that indicated we need to refresh the gain settings */
			refresh_gain = 0;
		}

		/* Sample and read data from SPI VibraTek sampling via SPI ADC */
		gpio_set_pin_level(PB05_SPInCS_CARDA, 0);

		command_dataw[0] = 0x60 + (ADC_CH0 << 2);
		bash_spi_transfer(command_dataw, command_datar, 2);
	
		gpio_set_pin_level(PB05_SPInCS_CARDA, 1);
		/* Completed sample and read data from SPI VibraTek sampling via SPI ADC */

		/* If Card A has a VibraTek or VibraPoint installed and the sample buffer is not yet full then... */
		if (((cardA_type == CARD_VIBRATEK) || (cardA_type == CARD_VIBRAPOINT)) && (card_sampleindex < SAMPLE_BUFFER_SIZE)) {
			cardAch0_samplebuffer[card_sampleindex] = command_datar[1];
		}

		/* Sample and read data from SPI VibraTek sampling via SPI ADC */
		gpio_set_pin_level(PB05_SPInCS_CARDA, 0);

		command_dataw[0] = 0x60 + (ADC_CH1 << 2);
		bash_spi_transfer(command_dataw, command_datar, 2);

		gpio_set_pin_level(PB05_SPInCS_CARDA, 1);
		/* Completed sample and read data from SPI VibraTek sampling via SPI ADC */

		if (((cardA_type == CARD_VIBRATEK) || (cardA_type == CARD_VIBRAPOINT)) && (card_sampleindex < SAMPLE_BUFFER_SIZE)) {
			cardAch1_samplebuffer[card_sampleindex] = command_datar[1];
		}

		if (((cardA_type == CARD_VIBRATEK) || (cardA_type == CARD_VIBRAPOINT)) && (!card_sampleindex)) {
			/* Sample and read data from SPI VibraTek sampling via SPI ADC */
			gpio_set_pin_level(PB05_SPInCS_CARDA, 0);

			command_dataw[0] = 0x60 + (ADC_CH2 << 2);
			bash_spi_transfer(command_dataw, command_datar, 2);
					
			gpio_set_pin_level(PB05_SPInCS_CARDA, 1);
			/* Completed sample and read data from SPI VibraTek sampling via SPI ADC */

			cardAch2_samplebuffer = command_datar[1];

			/* Sample and read data from SPI VibraTek sampling via SPI ADC */
			gpio_set_pin_level(PB05_SPInCS_CARDA, 0);

			command_dataw[0] = 0x60 + (ADC_CH3 << 2);
			bash_spi_transfer(command_dataw, command_datar, 2);

			gpio_set_pin_level(PB05_SPInCS_CARDA, 1);
			/* Completed sample and read data from SPI VibraTek sampling via SPI ADC */

			cardAch3_samplebuffer = command_datar[1];
		}
		
		/* If Card A has a Maxi IO installed then... */
		if (cardA_type == CARD_MAXI_IO)  {
			// TODO

		/* If Card A has a Mini IO installed then... */
		} else if (cardA_type == CARD_MINI_IO)  {
			if (miniA_chan) {
				if ((err = I2C_readMONITOREDINPUT(PC27_CARDA_I2C_SDA,  PC28_CARDA_I2C_CLK, &miniIO_A1_adcH,  &miniIO_A1_adcL))) {
					xprintf("Could NOT read from the Mini-IO ADC on CARDA:I2C [%i]\r\n", err);
				}
				
				if ((err = I2C_configMONITOREDINPUTS(PC27_CARDA_I2C_SDA,  PC28_CARDA_I2C_CLK, 0))) {
					xprintf("Could NOT write to the Mini-IO ADC controller on CARDA:I2C [%i]\r\n", err);
				}
			} else {
				if ((err = I2C_readMONITOREDINPUT(PC27_CARDA_I2C_SDA,  PC28_CARDA_I2C_CLK, &miniIO_A0_adcH,  &miniIO_A0_adcL))) {
					xprintf("Could NOT read from the Mini-IO ADC on CARDA:I2C [%i]\r\n", err);
				}
				
				if ((err = I2C_configMONITOREDINPUTS(PC27_CARDA_I2C_SDA,  PC28_CARDA_I2C_CLK, 1))) {
					xprintf("Could NOT write to the Mini-IO ADC controller on CARDA:I2C [%i]\r\n", err);
				}
			}

			if ((err = I2C_setEXPANDER(PC27_CARDA_I2C_SDA,  PC28_CARDA_I2C_CLK, miniIO_A_relay))) {
				xprintf("Could NOT write to the Mini-IO EXPANDER on CARDA:I2C [%i]\r\n", err);
			}

			if ((err = I2C_readEXPANDER(PC27_CARDA_I2C_SDA,  PC28_CARDA_I2C_CLK, &miniIO_A_inputs))) {
				xprintf("Could NOT read from the Mini-IO EXPANDER on CARDA:I2C [%i]\r\n", err);
			}
		/* If Card A has P&E installed then... */
		} else if (cardA_type == CARD_PE)  {
		// TODO
		/* If Card A has nothing installed then... */
		} else if (cardA_type == CARD_NOTFITTED)  {
		}

		/* Sample and read data from SPI VibraTek sampling via SPI ADC */
		gpio_set_pin_level(PB06_SPInCS_CARDB, 0);

		command_dataw[0] = 0x60 + (ADC_CH0 << 2);
		bash_spi_transfer(command_dataw, command_datar, 2);
				
		gpio_set_pin_level(PB06_SPInCS_CARDB, 1);
		/* Completed sample and read data from SPI VibraTek sampling via SPI ADC */

		/* If Card B has a VibraTek or VibraPoint installed and the sample buffer is not yet full then... */
		if (((cardB_type == CARD_VIBRATEK) || (cardB_type == CARD_VIBRAPOINT)) && (card_sampleindex < SAMPLE_BUFFER_SIZE)) {
			cardBch0_samplebuffer[card_sampleindex] = command_datar[1];
		}

		/* Sample and read data from SPI VibraTek sampling via SPI ADC */
		gpio_set_pin_level(PB06_SPInCS_CARDB, 0);

		command_dataw[0] = 0x60 + (ADC_CH1 << 2);
		bash_spi_transfer(command_dataw, command_datar, 2);
				
		gpio_set_pin_level(PB06_SPInCS_CARDB, 1);
		/* Completed sample and read data from SPI VibraTek sampling via SPI ADC */

		if (((cardB_type == CARD_VIBRATEK) || (cardB_type == CARD_VIBRAPOINT)) && (card_sampleindex < SAMPLE_BUFFER_SIZE)) {
			cardBch1_samplebuffer[card_sampleindex] = command_datar[1];
		}

		if (((cardB_type == CARD_VIBRATEK) || (cardB_type == CARD_VIBRAPOINT)) && (!card_sampleindex)) {
			/* Sample and read data from SPI VibraTek sampling via SPI ADC */
			gpio_set_pin_level(PB06_SPInCS_CARDB, 0);

			command_dataw[0] = 0x60 + (ADC_CH2 << 2);
			bash_spi_transfer(command_dataw, command_datar, 2);
	
			gpio_set_pin_level(PB06_SPInCS_CARDB, 1);
			/* Completed sample and read data from SPI VibraTek sampling via SPI ADC */

			cardBch2_samplebuffer = command_datar[1];

			/* Sample and read data from SPI VibraTek sampling via SPI ADC */
			gpio_set_pin_level(PB06_SPInCS_CARDB, 0);

			command_dataw[0] = 0x60 + (ADC_CH3 << 2);
			bash_spi_transfer(command_dataw, command_datar, 2);

			gpio_set_pin_level(PB06_SPInCS_CARDB, 1);
			/* Completed sample and read data from SPI VibraTek sampling via SPI ADC */

			cardBch3_samplebuffer = command_datar[1];
		}
		
		/* If Card B has a Maxi IO installed then... */
		if (cardB_type == CARD_MAXI_IO)  {
			// TODO
		/* If Card B has a Mini IO installed then... */
		} else if (cardB_type == CARD_MINI_IO)  {
			if (miniB_chan) {
				if ((err = I2C_readMONITOREDINPUT(PB24_CARDB_I2C_SDA,  PB25_CARDB_I2C_CLK, &miniIO_B1_adcH,  &miniIO_B1_adcL))) {
					xprintf("Could NOT read from the Mini-IO ADC on CARDB:I2C [%i]\r\n", err);
				}
				
				if ((err = I2C_configMONITOREDINPUTS(PB24_CARDB_I2C_SDA,  PB25_CARDB_I2C_CLK, 0))) {
					xprintf("Could NOT write to the Mini-IO ADC controller on CARDB:I2C [%i]\r\n", err);
				}
			} else {
				if ((err = I2C_readMONITOREDINPUT(PB24_CARDB_I2C_SDA,  PB25_CARDB_I2C_CLK, &miniIO_B0_adcH,  &miniIO_B0_adcL))) {
					xprintf("Could NOT read from the Mini-IO ADC on CARDB:I2C [%i]\r\n", err);
				}
				
				if ((err = I2C_configMONITOREDINPUTS(PB24_CARDB_I2C_SDA,  PB25_CARDB_I2C_CLK, 1))) {
					xprintf("Could NOT write to the Mini-IO ADC controller on CARDB:I2C [%i]\r\n", err);
				}
			}

			if ((err = I2C_setEXPANDER(PB24_CARDB_I2C_SDA,  PB25_CARDB_I2C_CLK, miniIO_B_relay))) {
				xprintf("Could NOT write to the Mini-IO EXPANDER on CARDB:I2C [%i]\r\n", err);
			}

			if ((err = I2C_readEXPANDER(PB24_CARDB_I2C_SDA,  PB25_CARDB_I2C_CLK, &miniIO_B_inputs))) {
				xprintf("Could NOT read from the Mini-IO EXPANDER on CARDB:I2C [%i]\r\n", err);
			}
		/* If Card B has nothing installed then... */
		} else if (cardB_type == CARD_NOTFITTED)  {
		}

		/* If any card slot has a VibraTek or VibraPoint installed then... */
		if ((cardA_type == CARD_VIBRATEK) || (cardB_type == CARD_VIBRATEK) || (cardA_type == CARD_VIBRAPOINT) || (cardB_type == CARD_VIBRAPOINT)) {
			/* Increment the sample index pointer */
			card_sampleindex++;
			
			/* When the buffers are full, it is time to signal that we need to send the UDP packets */
			if ((card_sampleindex == SAMPLE_BUFFER_SIZE) && (readenvironment == 1)) {
				readenvironment = 2;
				read_boardvalues();
			}
		} else if (readenvironment == 1) {
			/* If we just want to indicate that the board live values need to be read...*/
			readenvironment = 2;
			read_boardvalues();
		}

		/* Every 0.5 seconds, we need to check BPDU packets and ring topology state-machines... */
		if (ring_timer) {
			/* Go call the handler */
			if (ring_check(bpdu_arrived, &stormstate, &ring)) {
				/* If we need to send a UDP FLUSH MAC table packet, then set this flag */
				send_relearn_udp = 1;
			}

			/* De-assert timer flag */
			ring_timer = 0;

			/* If we are configured as a base-station, update the global variable containing the Ethernet ring topology state */
			if (((CONFIG *)&settings_buffer)->loop_basestation) {
				ring_broken = (ring) ? RING_BROKEN : RING_CLOSED;
			} else {
				ring_broken = RING_NOTCONFIGURED;
			}
		}

		key_check++;
		if (key_check < 3000) {
			continue;
		}
		
		key_check = 0;
		
		/* check for a character being pressed on the CLI keyboard */
		if (io_read((struct io_descriptor *const)io, (uint8_t *const)&ch, 1)) {
			/* If so, then read and get the character pressed */
			key = (unsigned char)ch;

			switch (key) {
				case 'r':
				case 'R':
				xprintf("\r\n. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .\r\n");
				xprintf("<CLI DEBUG> [REBOOT] command\r\n");
				xprintf(". . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .\r\n\r\n");

				_reset_mcu();
				break;

				case 'd':
				case 'D':
				/* [DEFAULT CONFIG] command */
				xprintf("\r\n. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .\r\n");
				xprintf("<CLI DEBUG> [CONFIGURATION DEFAULT] command\r\n");

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
				
				xprintf(". . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .\r\n\r\n");
				break;

				case 'X':
				case 'x':
				/* [ID RESET] command */
				xprintf("\r\n. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .\r\n");
				xprintf("<CLI DEBUG> [ID RESET] command\r\n", key);

				/* Re-write default values ready for write-back */
				((CONFIG *)&settings_buffer)->ID = 0x00000000;

				/* Write these values back to the EEprom storage area */
				if (EEprom_settings(settings_buffer, SETTING_STRUCTURE_SIZE, 1) != ERR_NONE) {
					xprintf("MultiSense [configuration ERROR WRITING]\r\n");
					} else {
					xprintf("MultiSense [configuration WRITTEN]\r\n");
				}
				
				xprintf("Rebooting...\r\n");
				_reset_mcu();
				break;

				case 'b':
				case 'B':
				/* [BASE STATION] command */
				xprintf("\r\n. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .\r\n");
				xprintf("<CLI DEBUG> [BASESTATION] command\r\n");

				/* Re-write default values ready for write-back */
				((CONFIG *)&settings_buffer)->loop_basestation = 1;

				/* Write these values back to the EEprom storage area */
				if (EEprom_settings(settings_buffer, SETTING_STRUCTURE_SIZE, 1) != ERR_NONE) {
					xprintf("MultiSense [configuration ERROR WRITING]\r\n");
					} else {
					xprintf("MultiSense [configuration WRITTEN]\r\n");
				}
				
				xprintf(". . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .\r\n\r\n");
				break;

				case 'p':
				case 'P':
				/* [PASS THROUGH] command */
				xprintf("\r\n. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .\r\n");
				xprintf("<CLI DEBUG> [PASS THROUGH] command\r\n");

				/* Re-write default values ready for write-back */
				((CONFIG *)&settings_buffer)->loop_basestation = 0;

				/* Write these values back to the EEprom storage area */
				if (EEprom_settings(settings_buffer, SETTING_STRUCTURE_SIZE, 1) != ERR_NONE) {
					xprintf("MultiSense [configuration ERROR WRITING]\r\n");
					} else {
					xprintf("MultiSense [configuration WRITTEN]\r\n");
				}
				
				xprintf(". . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .\r\n\r\n");
				break;

				case 'l':
				case 'L':
				/* [LIVE BOARD PRINT] command */
				xprintf("\r\n. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .");
				xprintf("\r\n");
				xprintf("<CLI DEBUG> [LIVE BOARD PRINT] command");
				xprintf("\r\n");
				xprintf("<CLI DEBUG> ID               = %d", ((CONFIG *)&settings_buffer)->ID);
				xprintf("\r\n");
				xprintf("<CLI DEBUG> Name             = %s", ((CONFIG *)&settings_buffer)->name);
				xprintf("\r\r\r\r\r\n");
				xprintf("<CLI DEBUG> gainA            = %d", ((CONFIG *)&settings_buffer)->gain_cardA);
				xprintf("\r\n");
				xprintf("<CLI DEBUG> gainB            = %d", ((CONFIG *)&settings_buffer)->gain_cardB);
				xprintf("\r\n");
				xprintf("<CLI DEBUG> Mode             = %s", (((CONFIG *)&settings_buffer)->loop_basestation) ? "BASESTATION" : "PASS-THROUGH");
				xprintf("\r\r\r\r\r\n");
				xprintf("<CLI DEBUG> Water1           = %d", (int)readdata_water1);
				xprintf("\r\n");
				xprintf("<CLI DEBUG> Water2           = %d", (int)readdata_water2);
				xprintf("\r\n");
				xprintf("<CLI DEBUG> HV               = %d", (int)highvoltage);
				xprintf("\r\n");
				
				if ((readdata_tempmoisture[1] == 0xFF) && (readdata_tempmoisture[0] == 0xFF)) {
					xprintf("<CLI DEBUG> Temp             = ERROR");
				} else if ((((((((int)readdata_tempmoisture[0] * 256)) + ((int)readdata_tempmoisture[1])) * 165) / 65536) - 40) >= 0) {
					xprintf("<CLI DEBUG> Temp             = %dC", ((((((int)readdata_tempmoisture[0] * 256)) + ((int)readdata_tempmoisture[1])) * 165) / 65536) - 40);
				} else {
					xprintf("<CLI DEBUG> Temp             = -%dC", -(((((((int)readdata_tempmoisture[0] * 256)) + ((int)readdata_tempmoisture[1])) * 165) / 65536) - 40));
				}
			
				xprintf("\r\n");
				
				if ((readdata_tempmoisture[3] == 0xFF) && (readdata_tempmoisture[2] == 0xFF)) {
					xprintf("<CLI DEBUG> Humidity         = ERROR");
					} else {
					xprintf("<CLI DEBUG> Humidity         = %d%c", ((int)((((int)readdata_tempmoisture[2] * 256)) + ((int)readdata_tempmoisture[3])) * 100) / 65536, '%');
				}
				
				xprintf("\r\n");
				xprintf("<CLI DEBUG> Tamper           = %s", (tamper) ? "UP" : "DOWN");
				xprintf("\r\r\r\r\n");
				xprintf("<CLI DEBUG> Port1            = %s", (link_port1) ? "LINK OK" : "LINK DOWN");
				xprintf("\r\r\r\r\n");
				xprintf("<CLI DEBUG> Port2            = %s", (link_port2) ? "LINK OK" : "LINK DOWN");
				xprintf("\r\r\r\r\n");
				xprintf("<CLI DEBUG> Port3            = %s", (link_port3) ? "LINK OK" : "LINK DOWN");
				xprintf("\r\r\r\r\n");
				xprintf("<CLI DEBUG> CardA            = %s", (cardA_type == CARD_NOTFITTED) ? "NOT FITTED" : (cardA_type == CARD_VIBRAPOINT) ? "VibraPoint" : (cardA_type == CARD_VIBRATEK) ? "VibraTek" :
					                                             (cardA_type == CARD_MAXI_IO) ? "Maxi IO" : (cardA_type == CARD_MINI_IO) ? "Mini IO" : (cardA_type == CARD_PE) ? "P&E" : "Unknown");
				xprintf("\r\r\r\r\r\n");
				xprintf("<CLI DEBUG> CardB            = %s", (cardB_type == CARD_NOTFITTED) ? "NOT FITTED" : (cardB_type == CARD_VIBRAPOINT) ? "VibraPoint" : (cardB_type == CARD_VIBRATEK) ? "VibraTek" :
																 (cardB_type == CARD_MAXI_IO) ? "Maxi IO" : (cardB_type == CARD_MINI_IO) ? "Mini IO" : "Unknown");
				xprintf("\r\r\r\r\r\n");
				xprintf("<CLI DEBUG> Ring Topology    = %s", (ring_broken == RING_BROKEN) ? "BROKEN" : (ring_broken == RING_CLOSED) ? "LOOP" : "N/A");
				xprintf("\r\r\r\r\r\n");
				xprintf("<CLI DEBUG> Firmware         = %d.%d", MULTISENSE_FIRMWARE_MAJOR, MULTISENSE_FIRMWARE_MINOR);
				xprintf("\r\n");
				xprintf(". . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .\r\n\r\n");
				break;
				
				case 'i':
				case 'I':
				/* [IP SCHEME / NETWORK] command */
				xprintf("\r\n. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .\r\n");
				xprintf("<CLI DEBUG> [IP SCHEME / NETWORK] command\r\n");
				xprintf("<CLI DEBUG> IP               = %d.%d.%d.%d\r\n", uip_ipaddr1(ipaddr), uip_ipaddr2(ipaddr), uip_ipaddr3(ipaddr), uip_ipaddr4(ipaddr));
				xprintf("<CLI DEBUG> Netmask          = %d.%d.%d.%d\r\n", uip_ipaddr1(netmask), uip_ipaddr2(netmask), uip_ipaddr3(netmask), uip_ipaddr4(netmask));
				xprintf("<CLI DEBUG> Gateway          = %d.%d.%d.%d\r\n", uip_ipaddr1(gwaddr), uip_ipaddr2(gwaddr), uip_ipaddr3(gwaddr), uip_ipaddr4(gwaddr));
				xprintf("<CLI DEBUG> MAC              = %x:%x:%x:%x:%x:%x\r\n", mac_raw[0], mac_raw[1], mac_raw[2], mac_raw[3], mac_raw[4], mac_raw[5]);
				xprintf(". . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .\r\n\r\n");
				break;

				case 's':
				case 'S':
				/* [SPEED MAINLOOP] command */
				xprintf("\r\n. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .\r\n");
				xprintf("<CLI DEBUG> [SPEED MAINLOOP] command\r\n");
				xprintf("<CLI DEBUG> Main-loop/Sampling = %d\r\n", looprate);
				xprintf(". . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .\r\n\r\n");
				break;

				/* All unrecognized command key-strokes dealt with here (with a HELP screen included) */
				default:
				xprintf("\r\n. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .\r\n");
				xprintf("<CLI DEBUG> Unknown command\r\n\r\nHelp Screen:\r\n\r\n");
				xprintf("R         [REBOOT] command\r\n");
				xprintf("D         [CONFIGURATION DEFAULT] command\r\n");
				xprintf("X         [ID RESET] command\r\n");
				xprintf("B         [BASESTATION] command\r\n");
				xprintf("P         [PASS THROUGH] command\r\n");
				xprintf("L         [LIVE BOARD PRINT] command\r\n");
				xprintf("I         [IP SCHEME / NETWORK] command\r\n");
				xprintf("S         [SPEED MAINLOOP] command\r\n\r\n");
				xprintf(". . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .\r\n\r\n");
				break;
			}
		}
	}
}

// *****************************************************************************************************************************************************************
// Function:    tx_callb(const struct usart_async_descriptor *const descr)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: INTERCEPT HOOK STUB for <UART_1>
// Returns:     Nothing (NULL)
// *****************************************************************************************************************************************************************
void tx_callb(const struct usart_async_descriptor *const descr)
{
}

// *****************************************************************************************************************************************************************
// Function:    rx_callb(const struct usart_async_descriptor *const descr)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: INTERCEPT HOOK STUB for <UART_1>
// Returns:     Nothing (NULL)
// *****************************************************************************************************************************************************************
void rx_callb(const struct usart_async_descriptor *const descr)
{
}

// *****************************************************************************************************************************************************************
// Function:    main(void)
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Description: Main start-point of whole application code
// Returns:     Nothing (NULL)
// *****************************************************************************************************************************************************************
int main(void)
{
	uint8_t			ch;
	unsigned char	ID_index;
	unsigned char	ID_string[9];
	unsigned int	new_id;
			
	/* Initialize the lower-level Atmel drivers, hardware and HAL interface */
	atmel_start_init();

	usart_async_get_io_descriptor(&USART_1, &io);
	usart_async_register_callback(&USART_1, USART_ASYNC_TXC_CB, tx_callb);
	usart_async_register_callback(&USART_1, USART_ASYNC_RXC_CB, rx_callb);
	usart_async_enable(&USART_1);
	
	hri_adc_set_DSEQCTRL_INPUTCTRL_bit(ADC0);
	_dma_set_source_address(DMA_CHANNEL_0, (const void *const)inputctrl_buff);
	_dma_set_destination_address(DMA_CHANNEL_0, (void *)(uint32_t) &(ADC0->DSEQDATA.reg));
	_dma_set_data_amount(DMA_CHANNEL_0, SEQ_LENGTH);
	_dma_enable_transaction(DMA_CHANNEL_0, false);
	
	xprintf("\r\n\r\n\r\n---------------------\r\nBOOT-UP\r\n---------------------\r\n\r\n");
	
	/* Clear and initialize the global variables */
	var_init();

	/* Set-up the none-hardware utilized GPIO pins */
	gpio_init();

	/* Set-up the SPI bus and the USB host stack */
	comms_init();

	/* Reset the Ethernet SWITCH IC via GPIO reset line */
	switch_init();
	
	/* Initialize the Ethernet SWITCH IC via the SPI bus */
	switch_configure();

	/* Define and start the main timer as 1/10th of a second ticker */
	timer_setup();

	/* Internal on-chip ADC feature initialization, used for the water detection feature */
	ADC_init();

	/* Enable and configure the Cryptography on-chip engine with the AES 128-bit private-key */
	crypto_init();

	/* IP stack initialization */
	uip_init();																									// initialize the IP stack

	/* Set the board MAC and IP address along with the network schemes */
	address_configure();
	
	if (((CONFIG *)&settings_buffer)->ID > 0x00000000) {
		/* Initialize the ring loop topology function */
		ring_init((unsigned char)(((CONFIG *)&settings_buffer)->loop_basestation), (unsigned char *)&mac_raw);
	} else {
		xprintf("********************************************************************************************\r\n");
		xprintf("* THE MULTI-SENSE DEVICE [ID] HAS NOT BEEN SET. THIS DEFICE WILL NOT FUNCTION OR RUN AS    *\r\n");
		xprintf("* NORMAL UNTIL THE [ID] NUMBER HAS BEEN ENTERED BY THIS TERMINAL FACILITY.                 *\r\n");
		xprintf("* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  *\r\n");
		xprintf("* PLEASE ENTER THE 32-BIT DEVICE [ID] BELOW                                                *\r\n");
		xprintf("********************************************************************************************\r\n");
	}
	
	if (((CONFIG *)&settings_buffer)->ID > 0x00000000) {
		xprintf("MultiSense [boot-up & init completed ID = %d]\r\n", ((CONFIG *)&settings_buffer)->ID);
		xprintf("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\r\n");
	} else {
		do {
			xprintf("\r\nEnter new [ID] now: ");
			/* check for a character being pressed on the CLI keyboard */
					
			ID_index = 0;
					
			while (1) {
				delay_ms(100);
				
				if (io_read((struct io_descriptor *const)io, (uint8_t *const)&ch, 1)) {
					/* If so, then read and get the character pressed */
				
					if ((unsigned char)ch == 10) {
						continue;
					}
				
					if ((unsigned char)ch == 13) {
						break;
					}
				
					if ((unsigned char)ch < '0') {
						continue;
					}
				
					if ((unsigned char)ch > '9') {
						continue;
					}
				
					if (ID_index == 8) {
						continue;
					}
				
					ID_string[ID_index++] = (unsigned char)ch;
					xprintf("%c", ch);
				}
			}
			
			ID_string[ID_index] = 0;
			xprintf("ID entered: %s\r\n", ID_string);
			xprintf("\r\nIs this correct Y/N?\r\n");
			
			while (1) {
				if (io_read((struct io_descriptor *const)io, (uint8_t *const)&ch, 1)) {
				/* If so, then read and get the character pressed */
				
					if (((unsigned char)ch == 'y') || ((unsigned char)ch == 'Y')) {
						sscanf((char *)ID_string, "%u", &new_id);
		
						if (!new_id) {
							xprintf("\r\nERROR ID cannot be <ZERO>!!!!!\r\n");
							break;
						}
		
						/* Re-write default values ready for write-back */
						((CONFIG *)&settings_buffer)->ID = new_id;

						/* Write these values back to the EEprom storage area */
						if (EEprom_settings(settings_buffer, SETTING_STRUCTURE_SIZE, 1) != ERR_NONE) {
							xprintf("MultiSense [configuration ERROR WRITING]\r\n");
						} else {
							xprintf("MultiSense [configuration WRITTEN]\r\n");
						}
									
						xprintf("\r\nRebooting...(please wait)\r\n\r\n");
					   _reset_mcu();
					} else if (((unsigned char)ch == 'n') || ((unsigned char)ch == 'N')) {
						break;
					}
				}
			}
		}
		while (1);
	}
	
	/* Create, enable and start the system watchdog (4 seconds timeout period) */
	watchdog_init();
	
	/* Run the main-loop */
	main_loop();

	/* Code should never return to here */
	return 0;
}