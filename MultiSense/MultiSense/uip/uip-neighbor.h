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

#ifndef __UIP_NEIGHBOR_H__
#define __UIP_NEIGHBOR_H__

#include "uip.h"

struct uip_neighbor_addr {
#if UIP_NEIGHBOR_CONF_ADDRTYPE
  UIP_NEIGHBOR_CONF_ADDRTYPE addr;
#else
  struct uip_eth_addr addr;
#endif
};

void uip_neighbor_init(void);
void uip_neighbor_add(uip_ipaddr_t ipaddr, struct uip_neighbor_addr *addr);
void uip_neighbor_update(uip_ipaddr_t ipaddr);
struct uip_neighbor_addr *uip_neighbor_lookup(uip_ipaddr_t ipaddr);
void uip_neighbor_periodic(void);

#endif /* __UIP-NEIGHBOR_H__ */