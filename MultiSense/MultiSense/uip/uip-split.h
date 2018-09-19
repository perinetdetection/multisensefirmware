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

#ifndef __UIP_SPLIT_H__
#define __UIP_SPLIT_H__

/**
 * Handle outgoing packets.
 *
 * This function inspects an outgoing packet in the uip_buf buffer and
 * sends it out using the uip_fw_output() function. If the packet is a
 * full-sized TCP segment it will be split into two segments and
 * transmitted separately. This function should be called instead of
 * the actual device driver output function, or the uip_fw_output()
 * function.
 *
 * The headers of the outgoing packet is assumed to be in the uip_buf
 * buffer and the payload is assumed to be wherever uip_appdata
 * points. The length of the outgoing packet is assumed to be in the
 * uip_len variable.
 *
 */
void uip_split_output(void);

#endif /* __UIP_SPLIT_H__ */