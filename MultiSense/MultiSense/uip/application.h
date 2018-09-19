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

#ifndef __APPLICATION_H__
#define __APPLICATION_H__

#include "uipopt.h"
#include "psock.h"

typedef struct udp_application_state {
  struct psock	p;
  char			inputbuffer[10];
  char			name[40];
} uip_udp_appstate_t;

typedef struct tcp_application_state {
	struct psock	p;
	char			inputbuffer[10];
	char			name[40];
} uip_tcp_appstate_t;

/* Finally we define the application function to be called by uIP. */
extern void application_appcall(void);
extern void application_udp_appcall(void);

#ifndef UIP_APPCALL
#define UIP_APPCALL application_appcall
#endif /* UIP_APPCALL */

#ifndef UIP_UDP_APPCALL
#define UIP_UDP_APPCALL application_udp_appcall
#endif /* UIP_APPCALL */

void application_init(void);
void tcpip_output(void);

#endif /* __APPLICATION_H__ */