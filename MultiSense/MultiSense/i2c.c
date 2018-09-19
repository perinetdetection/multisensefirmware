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
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hpl_adc_base.h>

#define I2C_DELAY		5																	// 5 micro-second delay

// * --------------------------------------------------------------------------------------------------------------------------------- 
// * MACRO:               I2C_init() : High Level									                     
// * USAGE:               Called by the application, the SDA and CLK bits are set to ready with default logic LOW		             
// * --------------------------------------------------------------------------------------------------------------------------------- 
#define I2C_init()                                                             				\
	gpio_set_pin_level(i2cclk, 1);  														\
	gpio_set_pin_level(i2csda, 1); 															\
	I2C_dirOP();																			\
	delay_us(I2C_DELAY)

// * --------------------------------------------------------------------------------------------------------------------------------- 
// * MACRO:               I2C_clocklo() : Low Level								                     
// * USAGE:               Called by Medium level macros, the CLK line is driven logic LOW then allowed to settle                       
// * --------------------------------------------------------------------------------------------------------------------------------- 
#define I2C_clocklo()  																		\
	gpio_set_pin_level(i2cclk, 0);  														\
    delay_us(I2C_DELAY)

// * --------------------------------------------------------------------------------------------------------------------------------- 
// * MACRO:               I2C_clockhi()  : Low Level								                     
// * USAGE:               Called by Medium level macros, the CLK line is driven logic HIGH then allowed to settle                      
// * --------------------------------------------------------------------------------------------------------------------------------- 
#define I2C_clockhi()  																		\
	gpio_set_pin_level(i2cclk, 1);															\
    delay_us(I2C_DELAY)

// * --------------------------------------------------------------------------------------------------------------------------------- 
// * MACRO:               I2C_datalo() : Low Level								                     
// * USAGE:               Called by Medium level macros, the SDA line is driven logic LOW then allowed to settle                       
// * --------------------------------------------------------------------------------------------------------------------------------- 
#define I2C_datalo()                                                           				\
	gpio_set_pin_level(i2csda, 0);															\
    delay_us(I2C_DELAY)

// * --------------------------------------------------------------------------------------------------------------------------------- 
// * MACRO:               I2C_datahi() : Low Level								                     
// * USAGE:               Called by Medium level macros, the SDA line is driven logic HIGH then allowed to settle                      
// * --------------------------------------------------------------------------------------------------------------------------------- 
#define I2C_datahi()                                                          		 		\
	gpio_set_pin_level(i2csda, 1); 															\
    delay_us(I2C_DELAY)

// * --------------------------------------------------------------------------------------------------------------------------------- 
// * MACRO:               I2C_dirOP() : Low Level									                     
// * USAGE:               Called by Medium level macros, the SDA line is set to be an OUTPUT then allowed to settle                    
// * --------------------------------------------------------------------------------------------------------------------------------- 
#define I2C_dirOP()                        	                            					\
    gpio_set_pin_direction(i2csda, GPIO_DIRECTION_OUT);										\
    delay_us(I2C_DELAY)

// * --------------------------------------------------------------------------------------------------------------------------------- 
// * MACRO:               I2C_dirIP() : Low Level									                     
// * USAGE:               Called by Medium level macros, the SDA line is set to be an INPUT then allowed to settle                     
// * --------------------------------------------------------------------------------------------------------------------------------- 
#define I2C_dirIP()                                                            				\
	gpio_set_pin_direction(i2csda, GPIO_DIRECTION_IN);										\
	delay_us(I2C_DELAY)

// * --------------------------------------------------------------------------------------------------------------------------------- 
// * MACRO:               I2C_OPEN(sda, clk) : High Level										     
// * USAGE:               Called by the application, this macro assigns the function pointers, addresses, and state machine            
// * --------------------------------------------------------------------------------------------------------------------------------- 
#define I2C_OPEN(sda, clk)           														\
	i2csda = sda;                                                          					\
	i2cclk = clk

// * --------------------------------------------------------------------------------------------------------------------------------- 
// * MACRO:               I2C_CLOSE() : High Level										     
// * USAGE:               Called by the application, this macro clears the function pointers, states, and addresses when done          
// * --------------------------------------------------------------------------------------------------------------------------------- 
#define I2C_CLOSE()                                                            				\
	i2csda = 0;                                                            					\
	i2cclk = 0

// * --------------------------------------------------------------------------------------------------------------------------------- 
// * MACRO:               I2C_SENDADDR(addr_dev, counter) : High Level							             
// * USAGE:               Called by the application, this macro sends the address of the I2C device on the bus in the data burst       
// * --------------------------------------------------------------------------------------------------------------------------------- 
#define I2C_SENDADDR(addr_dev, counter)                                        				\
	for (counter = 0x40; counter > 0;) {                                   		      		\
           	if (addr_dev & counter) { 														\
			I2C_datahi();                       											\
	 	}																					\
           	else { 																			\
			I2C_datalo(); 																	\
		}                                               									\
  		I2C_clockhi();                                           							\
   		I2C_clocklo();                                               						\
        counter = counter / 2;                                      						\
    }

// * --------------------------------------------------------------------------------------------------------------------------------- 
// * MACRO:               I2C_SENDDATA_(data, counter) : High Level						                     
// * USAGE:               Called by the application, this macro sends the 8-bits of data in the I2C data burst                   
// * --------------------------------------------------------------------------------------------------------------------------------- 
#define I2C_SENDDATA(data, counter)                                        		   		    \
    for (counter = 0x80; counter > 0;)  {                               					\
		if (data & counter) { 																\
			I2C_datahi();																	\
		}                                													\
           	else {																			\
			I2C_datalo();																	\
		}                                               									\
        I2C_clockhi();                                            							\
        I2C_clocklo();                                               						\
        counter = counter / 2;                                     							\
	}

// * --------------------------------------------------------------------------------------------------------------------------------- 
// * MACRO:               I2C_START(ret) : Medium Level                                                                                
// * USAGE:               Called by the application, this macro sends the start pulse of the I2C data burst                            
// * --------------------------------------------------------------------------------------------------------------------------------- 
#define I2C_START(ret)                                                         				\
	I2C_init();                                                            					\
	I2C_datalo();                                                          					\
	I2C_clocklo();

// * --------------------------------------------------------------------------------------------------------------------------------- 
// * MACRO:               I2C_WRITE() : Medium Level                                                                                   
// * USAGE:               Called by the application, this macro sets the I2C data burst to WRITE mode                                  
// * --------------------------------------------------------------------------------------------------------------------------------- 
#define I2C_WRITE()				                         									\
	I2C_datalo();                                                         					\
	I2C_clockhi();                                                        					\
	I2C_clocklo()

// * --------------------------------------------------------------------------------------------------------------------------------- 
// * MACRO:               I2C_READ() : Medium Level						                                     
// * USAGE:               Called by the application, this macro sets the I2C data burst to READ mode                                   
// * --------------------------------------------------------------------------------------------------------------------------------- 
#define I2C_READ()                                                             				\
	I2C_datahi();                                                          					\
	I2C_clockhi();                                                         					\
	I2C_clocklo()

// * --------------------------------------------------------------------------------------------------------------------------------- 
// * MACRO:               I2C_WAITACKOP(ret) : High Level										     
// * USAGE:               Called by the application, this waits for the ack signal from the I2C device when SDA is an OUTPUT	     
// * --------------------------------------------------------------------------------------------------------------------------------- 
#define I2C_WAITACKOP(ret)                                           		          		\
	I2C_dirIP();                                                      		     			\
	I2C_clockhi();                                                    		     			\
	if (gpio_get_pin_level(i2csda)) { 														\
		I2C_STOP();																			\
		I2C_CLOSE();																		\
		return ret;																			\
	}       																				\
	I2C_clocklo();                                                         					\
	I2C_dirOP()

// * --------------------------------------------------------------------------------------------------------------------------------- 
// * MACRO:               I2C_WAITACKIP(ret) : High Level									             
// * USAGE:               Called by the application, this waits for the ack signal from the I2C device when SDA is an INPUT            
// * --------------------------------------------------------------------------------------------------------------------------------- 
#define I2C_WAITACKIP(ret)                                                     				\
	I2C_dirIP();                                                           					\
	I2C_clockhi();                                                        			 		\
	if (gpio_get_pin_level(i2csda)) { 														\
		I2C_STOP();																			\
		I2C_CLOSE();																		\
		return ret;																			\
	}        																				\
	I2C_clocklo();

// * --------------------------------------------------------------------------------------------------------------------------------- 
// * MACRO:               I2C_SENDACK() : Medium Level							                             
// * USAGE:               Called by the application, this macro sends an acknowledgment when receiving I2C data                       
// * ---------------------------------------------------------------------------------------------------------------------------------
#define I2C_SENDACK()                                                          				\
	I2C_dirOP();                                                           					\
	I2C_datalo();                                                          					\
	I2C_clockhi();                                                         					\
	I2C_clocklo();                                                         					\
	I2C_dirIP()
	
// * ---------------------------------------------------------------------------------------------------------------------------------
// * MACRO:               I2C_SENDNACK() : Medium Level
// * USAGE:               Called by the application, this macro sends an negative-acknowledgment when receiving I2C data
// * ---------------------------------------------------------------------------------------------------------------------------------
#define I2C_SENDNACK()                                                          			\
	I2C_dirOP();                                                           					\
	I2C_datahi();                                                          					\
	I2C_clockhi();                                                         					\
	I2C_clocklo();                                                         					\
	I2C_dirIP()	

// * --------------------------------------------------------------------------------------------------------------------------------- 
// * MACRO:               I2C_STOP() : Medium Level										     
// * USAGE:               Called by the application, this macro sends a STOP signal on the I2C SDA and CLK bus when finished and done  
// * ---------------------------------------------------------------------------------------------------------------------------------
#define I2C_STOP()                                                             				\
	I2C_clockhi();                                                         					\
	I2C_datahi();                                                          					\
	I2C_dirOP()

// * --------------------------------------------------------------------------------------------------------------------------------- *
// * FUNCTION:            I2C_configMONITOREDINPUTS(uint8_t sda,  uint8_t clk, unsigned char channel)                                  *
// * RETURNS:             int: 0 for success                                                                                           *
// *                                                                                                                                   *
// * CALLED BY:			  main()                                                                                                       *
// * CALLS:		          No function. ALL macros defined in this function file.                                                       *
// * --------------------------------------------------------------------------------------------------------------------------------- *
int I2C_configMONITOREDINPUTS(uint8_t sda,  uint8_t clk, unsigned char channel)

{
	uint8_t     		i2csda, i2cclk;
	unsigned char		forloop;

	/* Clip channel for correct assignment */
    channel &= 0x03;

	/* Open an I2C channel, send address 0x68, then send the configuration values for setting up a conversion result */
	I2C_OPEN(sda, clk);
	I2C_START(-1);
	I2C_SENDADDR(0x68, forloop);
	I2C_WRITE();
	I2C_WAITACKOP(-2);
	I2C_SENDDATA(((channel << 5) + 0x80), forloop);
	I2C_WAITACKOP(-3);
	
	/* If successful, and code execution got here, the close the I2C channel */
	I2C_STOP();
	I2C_CLOSE();
	
	return 0;
}

// * --------------------------------------------------------------------------------------------------------------------------------- *
// * FUNCTION:            I2C_readMONITOREDINPUT(uint8_t sda,  uint8_t clk, unsigned char *inputH, unsigned char *inputL)              *
// * RETURNS:             int: 0 for success                                                                                           *
// *                                                                                                                                   *
// * CALLED BY:			  main()                                                                                                       *
// * CALLS:		          No function. ALL macros defined in this function file.                                                       *
// * --------------------------------------------------------------------------------------------------------------------------------- *
int I2C_readMONITOREDINPUT(uint8_t sda,  uint8_t clk, unsigned char *inputH, unsigned char *inputL)

{
	uint8_t     		i2csda, i2cclk;
	unsigned char		forloop;

	if ((!inputL) || (!inputH)) {
		return -1;
	}

	/* Open an I2C channel, send address 0x68, then read the input value */
	I2C_OPEN(sda, clk);
	I2C_START(-2);
	I2C_SENDADDR(0x68, forloop);
	I2C_READ();
	I2C_WAITACKIP(-3);

	*inputH = 0;

	/* Beginning of the for-loops that captures data and read the upper 8-bits of the input value */
	for (forloop = 0x80; forloop > 0;) {
		I2C_clockhi();
		if (gpio_get_pin_level(i2csda)) {
			(*inputH) |= forloop;
		}

		I2C_clocklo();
		forloop = forloop / 2;
	}
	
	I2C_SENDACK();
	*inputL = 0;

	/* Beginning of the for-loops that captures data and read the lower 8-bits of the input value */
	for (forloop = 0x80; forloop > 0;) {
		I2C_clockhi();
		if (gpio_get_pin_level(i2csda)) {
			(*inputL) |= forloop;
		}

		I2C_clocklo();
		forloop = forloop / 2;
	}
	
	I2C_SENDNACK();	
	I2C_STOP();
	I2C_CLOSE();
	/* If successful, and code execution got here, the close the I2C channel */
	
	return 0;
}

// * --------------------------------------------------------------------------------------------------------------------------------- *
// * FUNCTION:            I2C_configEXPANDER(uint8_t sda,  uint8_t clk)                                                                *
// * RETURNS:             int: 0 for success                                                                                           *
// *                                                                                                                                   *
// * CALLED BY:			  main()                                                                                                       *
// * CALLS:		          No function. ALL macros defined in this function file.                                                       *
// * --------------------------------------------------------------------------------------------------------------------------------- *
int I2C_configEXPANDER(uint8_t sda,  uint8_t clk)

{
	uint8_t     		i2csda, i2cclk;
	unsigned char		forloop;

	/* Open an I2C channel, send address 0x41, index register 0x01 then send the 3 configuration values for inversion, output default and the direction register */
	I2C_OPEN(sda, clk);
	I2C_START(-1);
	I2C_SENDADDR(0x41, forloop);
	I2C_WRITE();
	I2C_WAITACKOP(-2);
	I2C_SENDDATA(0x01, forloop);
	I2C_WAITACKOP(-3);
	I2C_SENDDATA(0x00, forloop);
	I2C_WAITACKOP(-4);

	I2C_START(-1);
	I2C_SENDADDR(0x41, forloop);
	I2C_WRITE();
	I2C_WAITACKOP(-2);
	I2C_SENDDATA(0x02, forloop);
	I2C_WAITACKOP(-3);
	I2C_SENDDATA(0x00, forloop);
	I2C_WAITACKOP(-4);
	
	I2C_START(-1);
	I2C_SENDADDR(0x41, forloop);
	I2C_WRITE();
	I2C_WAITACKOP(-2);
	I2C_SENDDATA(0x03, forloop);
	I2C_WAITACKOP(-3);
	I2C_SENDDATA(0xC0, forloop);
	I2C_WAITACKOP(-4);		
	
	/* If successful, and code execution got here, the close the I2C channel */
	I2C_STOP();
	I2C_CLOSE();
	
	return 0;
}

// * --------------------------------------------------------------------------------------------------------------------------------- *
// * FUNCTION:            I2C_setEXPANDER(uint8_t sda,  uint8_t clk, unsigned char relay)                                              *
// * RETURNS:             int: 0 for success                                                                                           *
// *                                                                                                                                   *
// * CALLED BY:			  main()                                                                                                       *
// * CALLS:		          No function. ALL macros defined in this function file.                                                       *
// * --------------------------------------------------------------------------------------------------------------------------------- *
int I2C_setEXPANDER(uint8_t sda,  uint8_t clk, unsigned char relay)

{
	uint8_t     		i2csda, i2cclk;
	unsigned char		forloop;

	/* Open an I2C channel, send address 0x41, index register 0x01 then send the relay output value */
	I2C_OPEN(sda, clk);
	I2C_START(-1);
	I2C_SENDADDR(0x41, forloop);
	I2C_WRITE();
	I2C_WAITACKOP(-2);
	I2C_SENDDATA(0x01, forloop);
	I2C_WAITACKOP(-3);
	I2C_SENDDATA(relay, forloop);
	I2C_WAITACKOP(-4);
	
	/* If successful, and code execution got here, the close the I2C channel */
	I2C_STOP();
	I2C_CLOSE();
	
	return 0;
}

// * --------------------------------------------------------------------------------------------------------------------------------- *
// * FUNCTION:            I2C_readEXPANDER(uint8_t sda,  uint8_t clk, unsigned char *inputs)                                           *
// * RETURNS:             int: 0 for success                                                                                           *
// *                                                                                                                                   *
// * CALLED BY:			  main()                                                                                                       *
// * CALLS:		          No function. ALL macros defined in this function file.                                                       *
// * --------------------------------------------------------------------------------------------------------------------------------- *
int I2C_readEXPANDER(uint8_t sda,  uint8_t clk, unsigned char *inputs)

{
	uint8_t     		i2csda, i2cclk;
	unsigned char		forloop;

    if (!inputs) {
		return -1;	
	}

	/* Open an I2C channel, send address 0x41, index register 0x00 then read the input value */
	I2C_OPEN(sda, clk);
	I2C_START(-2);
	I2C_SENDADDR(0x41, forloop);
	I2C_WRITE();
	I2C_WAITACKOP(-3);
	I2C_SENDDATA(0x00, forloop);
	I2C_WAITACKOP(-4);
	
	/* Then restart comms and send address 0x40 */
	I2C_START(-5);
	I2C_SENDADDR(0x41, forloop);
	I2C_READ();
	I2C_WAITACKIP(-6);

	*inputs = 0;

	/* Beginning of the for-loops that captures data and read the lower 8-bits of the input values */
	for (forloop = 0x80; forloop > 0;) {
		I2C_clockhi();
		if (gpio_get_pin_level(i2csda)) {
			(*inputs) |= forloop;
		}

		I2C_clocklo();
		forloop = forloop / 2;
	}

	I2C_STOP();
	I2C_CLOSE();
	/* If successful, and code execution got here, the close the I2C channel */
	
	return 0;
}

// * --------------------------------------------------------------------------------------------------------------------------------- *
// * FUNCTION:            I2C_setGAIN(uint8_t sda,  uint8_t clk, unsigned char dac)                                                    *
// * RETURNS:             int: 0 for success                                                                                           *
// *                                                                                                                                   *
// * CALLED BY:			  main()                                                                                                       *
// * CALLS:		          No function. ALL macros defined in this function file.                                                       *
// * --------------------------------------------------------------------------------------------------------------------------------- *
int I2C_setGAIN(uint8_t sda,  uint8_t clk, unsigned char dac)

{
  	uint8_t     		i2csda, i2cclk;
    unsigned char		forloop;

	/* Open an I2C channel, send address 0x60, index register 0x00 then send the 8-bit "dac" value */
	I2C_OPEN(sda, clk);
    I2C_START(-1);
    I2C_SENDADDR(0x60, forloop);
    I2C_WRITE();
    I2C_WAITACKOP(-2);
    I2C_SENDDATA(0x00, forloop);
    I2C_WAITACKOP(-3);
	I2C_SENDDATA(0x00, forloop);
	I2C_WAITACKOP(-3);
	
	/* If successful, and code execution got here, the close the I2C channel */
    I2C_STOP();
	I2C_CLOSE();
	
	return 0;
}

// * --------------------------------------------------------------------------------------------------------------------------------------------------- *
// * FUNCTION:            I2C_getTEMPandMOISTURE(uint8_t sda,  uint8_t clk, unsigned char *th, unsigned char *tl, unsigned char *hh, unsigned char *hl)  *
// * RETURNS:             int: 0 for success                                                                                                             *
// *                                                                                                                                                     *
// * CALLED BY:			  main()                                                                                                                         *
// * CALLS:		          No function. ALL macros defined in this function file.                                                                         *
// * --------------------------------------------------------------------------------------------------------------------------------------------------- *
int I2C_getTEMPandMOISTURE(uint8_t sda,  uint8_t clk, unsigned char *th, unsigned char *tl, unsigned char *hh, unsigned char *hl)

{
	uint8_t     		i2csda, i2cclk;
	unsigned char		forloop;
	
	/* Assert and bounds check the return parameter variables */
	if ((!th) || (!tl) || (!hh) || (!hl)) {
		return -1;
	}

	/* Open an I2C channel, send address 0x40, index register 0x00 */
	I2C_OPEN(sda, clk);
	I2C_START(-2);
	I2C_SENDADDR(0x40, forloop);
	I2C_WRITE();
	I2C_WAITACKOP(-3);
	I2C_SENDDATA(0x00, forloop);
	I2C_WAITACKOP(-4);
	
	/* Then restart comms and send address 0x40 */
	I2C_START(-5);
	I2C_SENDADDR(0x40, forloop);
	I2C_READ();
	I2C_WAITACKIP(-6);

	*tl = 0;

	/* Beginning of the for-loops that captures data and read the lower 8-bits of the temperature */
	for (forloop = 0x80; forloop > 0;) {
		I2C_clockhi();
	    if (gpio_get_pin_level(i2csda)) {
			(*tl) |= forloop;
		}

		I2C_clocklo();
		forloop = forloop / 2;
	}

	I2C_STOP();
	I2C_CLOSE();
	/* If successful, and code execution got here, the close the I2C channel */
	
	/* Open an I2C channel, send address 0x40, index register 0x01 */
	I2C_OPEN(sda, clk);
	I2C_START(-2);
	I2C_SENDADDR(0x40, forloop);
	I2C_WRITE();
	I2C_WAITACKOP(-3);
	I2C_SENDDATA(0x01, forloop);
	I2C_WAITACKOP(-4);
	
	/* Then restart comms and send address 0x40 */
	I2C_START(-5);
	I2C_SENDADDR(0x40, forloop);
	I2C_READ();
	I2C_WAITACKIP(-6);

	*th = 0;
	
	/* Beginning of the for-loops that captures data and read the upper 8-bits of the temperature */
	for (forloop = 0x80; forloop > 0;) {
		I2C_clockhi();
		if (gpio_get_pin_level(i2csda)) {
			(*th) |= forloop;
		}

		I2C_clocklo();
		forloop = forloop / 2;
	}

	I2C_STOP();
	I2C_CLOSE();
	/* If successful, and code execution got here, the close the I2C channel */
	
	/* Open an I2C channel, send address 0x40, index register 0x02 */
	I2C_OPEN(sda, clk);
	I2C_START(-2);
	I2C_SENDADDR(0x40, forloop);
	I2C_WRITE();
	I2C_WAITACKOP(-3);
	I2C_SENDDATA(0x02, forloop);
	I2C_WAITACKOP(-4);
	
	/* Then restart comms and send address 0x40 */
	I2C_START(-5);
	I2C_SENDADDR(0x40, forloop);
	I2C_READ();
	I2C_WAITACKIP(-6);

	*hl = 0;
	
	/* Beginning of the for-loops that captures data and read the lower 8-bits of the humidity */
	for (forloop = 0x80; forloop > 0;) {
		I2C_clockhi();
		if (gpio_get_pin_level(i2csda)) {
			(*hl) |= forloop;
		}

		I2C_clocklo();
		forloop = forloop / 2;
	}

	I2C_STOP();
	I2C_CLOSE();
	/* If successful, and code execution got here, the close the I2C channel */
	
	/* Open an I2C channel, send address 0x40, index register 0x03 */
	I2C_OPEN(sda, clk);
	I2C_START(-2);
	I2C_SENDADDR(0x40, forloop);
	I2C_WRITE();
	I2C_WAITACKOP(-3);
	I2C_SENDDATA(0x03, forloop);
	I2C_WAITACKOP(-4);
	
	/* Then restart comms and send address 0x40 */
	I2C_START(-5);
	I2C_SENDADDR(0x40, forloop);
	I2C_READ();
	I2C_WAITACKIP(-6);

	*hh = 0;
	
	/* Beginning of the for-loops that captures data and read the upper 8-bits of the humidity */
	for (forloop = 0x80; forloop > 0;) {
		I2C_clockhi();
		if (gpio_get_pin_level(i2csda)) {
			(*hh) |= forloop;
		}

		I2C_clocklo();
		forloop = forloop / 2;
	}

	I2C_STOP();
	I2C_CLOSE();
	/* If successful, and code execution got here, the close the I2C channel */
	
	return 0;
}

// * --------------------------------------------------------------------------------------------------------------------------------- *
// * FUNCTION:            I2C_check_deviceID(uint8_t sda,  uint8_t clk, unsigned char deviceID)                                        *
// * RETURNS:             int: 0 for success                                                                                           *
// *                                                                                                                                   *
// * CALLED BY:			  main()                                                                                                       *
// * CALLS:		          No function. ALL macros defined in this function file.                                                       *
// * --------------------------------------------------------------------------------------------------------------------------------- *
int I2C_check_deviceID(uint8_t sda,  uint8_t clk, unsigned char deviceID)

{
	uint8_t     		i2csda, i2cclk;
    unsigned char		forloop;

    /* Open an I2C channel, send address held in parameter variable "deviceID" */
	I2C_OPEN(sda, clk);
	I2C_START(-1);
	I2C_SENDADDR(deviceID, forloop);
	I2C_READ();
	I2C_WAITACKOP(-2);
	/* If not successful, then function will return with < 0 value if adddress not found */
	
	I2C_STOP();
	I2C_CLOSE();
	
	/* If successful, and code execution got here, the close the I2C channel and return with ZERO value if address found */
	return 0;
}