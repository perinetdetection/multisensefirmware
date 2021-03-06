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

#ifndef STDIO_MAIN_H
#define STDIO_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdio.h>

#include <stdio.h>
#include <stdio_io.h>

/* Micrel Ethernet SWITCH SPI register definitions */
#define SPI_KSZ8794_FAMILY_ID		0x00
#define SPI_KSZ8794_START			0x01
#define SPI_KSZ8794_GLOBAL0			0x02
#define SPI_KSZ8794_GLOBAL1			0x03
#define SPI_KSZ8794_GLOBAL2			0x04
#define SPI_KSZ8794_GLOBAL3			0x05
#define SPI_KSZ8794_GLOBAL4			0x06
#define SPI_KSZ8794_GLOBAL5			0x07
#define SPI_KSZ8794_GLOBAL6			0x08
#define SPI_KSZ8794_GLOBAL9			0x0B
#define SPI_KSZ8794_GLOBAL10		0x0C
#define SPI_KSZ8794_PDMC1			0x0E
#define SPI_KSZ8794_PDMC2			0x0F
#define SPI_KSZ8794_PORT1CONTROL0	0x10
#define SPI_KSZ8794_PORT2CONTROL0	0x20
#define SPI_KSZ8794_PORT3CONTROL0	0x30
#define SPI_KSZ8794_PORT4CONTROL0	0x50
#define SPI_KSZ8794_PORT1CONTROL1	0x11
#define SPI_KSZ8794_PORT2CONTROL1	0x21
#define SPI_KSZ8794_PORT3CONTROL1	0x31
#define SPI_KSZ8794_PORT4CONTROL1	0x51
#define SPI_KSZ8794_PORT1CONTROL2	0x12
#define SPI_KSZ8794_PORT2CONTROL2	0x22
#define SPI_KSZ8794_PORT3CONTROL2	0x32
#define SPI_KSZ8794_PORT4CONTROL2	0x52
#define SPI_KSZ8794_PORT1CONTROL3	0x13
#define SPI_KSZ8794_PORT2CONTROL3	0x23
#define SPI_KSZ8794_PORT3CONTROL3	0x33
#define SPI_KSZ8794_PORT4CONTROL3	0x53
#define SPI_KSZ8794_PORT1CONTROL4	0x14
#define SPI_KSZ8794_PORT2CONTROL4	0x24
#define SPI_KSZ8794_PORT3CONTROL4	0x34
#define SPI_KSZ8794_PORT4CONTROL4	0x54
#define SPI_KSZ8794_PORT1CONTROL5	0x15
#define SPI_KSZ8794_PORT2CONTROL5	0x25
#define SPI_KSZ8794_PORT3CONTROL5	0x35
#define SPI_KSZ8794_PORT4CONTROL5	0x55
#define SPI_KSZ8794_PORT4CONTROL6	0x56
#define SPI_KSZ8794_PORT1CONTROL7	0x17
#define SPI_KSZ8794_PORT2CONTROL7	0x27
#define SPI_KSZ8794_PORT3CONTROL7	0x37
#define SPI_KSZ8794_PORT1STATUS0	0x18
#define SPI_KSZ8794_PORT2STATUS0	0x28
#define SPI_KSZ8794_PORT3STATUS0	0x38
#define SPI_KSZ8794_PORT1STATUS1	0x19
#define SPI_KSZ8794_PORT2STATUS1	0x29
#define SPI_KSZ8794_PORT3STATUS1	0x39
#define SPI_KSZ8794_PORT1CONTROL8	0x1A
#define SPI_KSZ8794_PORT2CONTROL8	0x2A
#define SPI_KSZ8794_PORT3CONTROL8	0x3A
#define SPI_KSZ8794_PORT1LINKMD		0x1B
#define SPI_KSZ8794_PORT2LINKMD		0x2B
#define SPI_KSZ8794_PORT3LINKMD		0x3B
#define SPI_KSZ8794_PORT1CONTROL9	0x1C
#define SPI_KSZ8794_PORT2CONTROL9	0x2C
#define SPI_KSZ8794_PORT3CONTROL9	0x3C
#define SPI_KSZ8794_PORT1CONTROL10	0x1D
#define SPI_KSZ8794_PORT2CONTROL10	0x2D
#define SPI_KSZ8794_PORT3CONTROL10	0x3D
#define SPI_KSZ8794_PORT1STATUS2	0x1E
#define SPI_KSZ8794_PORT2STATUS2	0x2E
#define SPI_KSZ8794_PORT3STATUS2	0x3E
#define SPI_KSZ8794_PORT1STATUS3	0x1F
#define SPI_KSZ8794_PORT2STATUS3	0x2F
#define SPI_KSZ8794_PORT3STATUS3	0x3F
#define SPI_KSZ8794_ACCESS_CONTROL0	0x6E
#define SPI_KSZ8794_ACCESS_CONTROL1	0x6F
#define SPI_KSZ8794_ACCESS_DATA8	0x70
#define SPI_KSZ8794_ACCESS_DATA7	0x71
#define SPI_KSZ8794_ACCESS_DATA6	0x72
#define SPI_KSZ8794_ACCESS_DATA5	0x73
#define SPI_KSZ8794_ACCESS_DATA4	0x74
#define SPI_KSZ8794_ACCESS_DATA3	0x75
#define SPI_KSZ8794_ACCESS_DATA2	0x76
#define SPI_KSZ8794_ACCESS_DATA1	0x77
#define SPI_KSZ8794_ACCESS_DATA0	0x78
#define SPI_KSZ8794_PORT2CONTROL20	0xA3
#define SPI_KSZ8794_PORT1CONTROL12	0xB0
#define SPI_KSZ8794_PORT2CONTROL12	0xC0
#define SPI_KSZ8794_PORT3CONTROL12	0xD0
#define SPI_KSZ8794_PORT4CONTROL12	0xF0
#define SPI_KSZ8794_PORT1CONTROL13	0xB1
#define SPI_KSZ8794_PORT2CONTROL13	0xC1
#define SPI_KSZ8794_PORT3CONTROL13	0xD1
#define SPI_KSZ8794_PORT4CONTROL13	0xF1
#define SPI_KSZ8794_PORT1CONTROL14	0xB2
#define SPI_KSZ8794_PORT2CONTROL14	0xC2
#define SPI_KSZ8794_PORT3CONTROL14	0xD2
#define SPI_KSZ8794_PORT4CONTROL14	0xF2
#define SPI_KSZ8794_PORT1CONTROL15	0xB3
#define SPI_KSZ8794_PORT2CONTROL15	0xC3
#define SPI_KSZ8794_PORT3CONTROL15	0xD3
#define SPI_KSZ8794_PORT4CONTROL15	0xF3
#define SPI_KSZ8794_PORT1CONTROL16	0xB4
#define SPI_KSZ8794_PORT2CONTROL16	0xC4
#define SPI_KSZ8794_PORT3CONTROL16	0xD4
#define SPI_KSZ8794_PORT4CONTROL16	0xF4
#define SPI_KSZ8794_PORT1CONTROL17	0xB5
#define SPI_KSZ8794_PORT2CONTROL17	0xC5
#define SPI_KSZ8794_PORT3CONTROL17	0xD5
#define SPI_KSZ8794_PORT4CONTROL17	0xF5

/* MultiSense specific GPIO definitions */
#define PB00_KSZ_RESET				GPIO(GPIO_PORTB, 0)
#define PB01_DETECT_PE				GPIO(GPIO_PORTB, 1)
#define PB02_TAMP_OP				GPIO(GPIO_PORTB, 2)
#define PB03_LED_ETH				GPIO(GPIO_PORTB, 3)
#define PB04_LED_PWR				GPIO(GPIO_PORTB, 4)
#define PB05_SPInCS_CARDA			GPIO(GPIO_PORTB, 5)
#define PB06_SPInCS_CARDB			GPIO(GPIO_PORTB, 6)
#define PB07_SPInCS_KSZ8974			GPIO(GPIO_PORTB, 7)
#define PB24_CARDB_I2C_SDA			GPIO(GPIO_PORTB, 24)
#define PB25_CARDB_I2C_CLK			GPIO(GPIO_PORTB, 25)
#define PC00_CARDA_PRESENT			GPIO(GPIO_PORTC, 0)
#define PC01_CARDB_PRESENT			GPIO(GPIO_PORTC, 1)
#define PC06_GPIO1					GPIO(GPIO_PORTC, 6)
#define PC07_GPIO2					GPIO(GPIO_PORTC, 7)
#define PC10_GPIO3					GPIO(GPIO_PORTC, 10)
#define PC11_GPIO4					GPIO(GPIO_PORTC, 11)
#define PC12_GPIO5					GPIO(GPIO_PORTC, 12)
#define PC27_CARDA_I2C_SDA			GPIO(GPIO_PORTC, 27)
#define PC28_CARDA_I2C_CLK			GPIO(GPIO_PORTC, 28)

/* Global definitions for the firmware environment */
#define REG_SPI_READ				0x20
#define	REG_SPI_WRITE				0x00

#define SETTING_STRUCTURE_SIZE		128
#define SAMPLE_BUFFER_SIZE			684

#define ADC_CH0						0
#define ADC_CH1						1
#define ADC_CH2						2
#define ADC_CH3						3

#define RING_CLOSED					0
#define RING_BROKEN					1
#define RING_NOTCONFIGURED			2

#define ID_IDENT_1					0x37
#define ID_IDENT_2					0x9F
#define ID_IDENT_3					0x1B
#define ID_IDENT_4					0x61
#define	CONFIG_IDENTIFIER1			0xFE453676
#define	CONFIG_IDENTIFIER2			0x937DEABC
#define MAC_ADDR_00                 0x00
#define MAC_ADDR_01                 0x50
#define MAC_ADDR_02                 0xC2
#define MAC_ADDR_03                 0x24

#define MAIN_UDPSOCKET		    	4000
#define CARDA_UDPSOCKET				4001
#define CARDB_UDPSOCKET				4002
#define RING_MANAGEMENT_SOCKET		4003

/* Type list for the status of each of the daughter-card slots */
typedef enum {
	CARD_NOTFITTED = 0,
	CARD_VIBRATEK,
	CARD_MINI_IO,
	CARD_MAXI_IO,
	CARD_VIBRAPOINT
} CARD_TYPE;

/* Different states of enacting the ID program mode */
typedef enum {
	ASSIGNID_WAITFOR_TAMP = 0,
	ASSIGNID_FIRSTPRESS,
	ASSIGNID_FIRSTRELEASE,
	ASSIGNID_SECONDPRESS,
	ASSIGNID_SECONDRELEASE,
	ASSIGNID_THIRDPRESS,
	ASSIGNID_THIRDRELEASE,
	ASSIGNID_EXIT
} IDMODE_TYPE;

/* Main structure image-overlay of the EEprom configuration stored as non-volatile RAM */
typedef struct {
	unsigned int					pattern1;
	unsigned int					pattern2;
	unsigned char					name[32 + 1];
	uint8_t							gain_cardA;
	uint8_t							gain_cardB;
	unsigned char					loop_basestation;
	uint16_t     					ID;
} CONFIG;

/* Function proto-typing of the "printf()" redirection on COM2 serial connector */
void stdio_redirect_init(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* STDIO_MAIN_H */
