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

#include "uip.h"
#include "uiplib.h"

unsigned char
uiplib_ipaddrconv(char *addrstr, unsigned char *ipaddr)
{
  unsigned char tmp;
  char c;
  unsigned char i, j;

  tmp = 0;
  
  for(i = 0; i < 4; ++i) {
    j = 0;
    do {
      c = *addrstr;
      ++j;
      if(j > 4) {
	return 0;
      }
      if(c == '.' || c == 0) {
	*ipaddr = tmp;
	++ipaddr;
	tmp = 0;
      } else if(c >= '0' && c <= '9') {
	tmp = (tmp * 10) + (c - '0');
      } else {
	return 0;
      }
      ++addrstr;
    } while(c != '.' && c != 0);
  }
  return 1;
}