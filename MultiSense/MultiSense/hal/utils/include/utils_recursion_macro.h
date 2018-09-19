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

#ifndef _UTILS_RECURSION_MACRO_H
#define _UTILS_RECURSION_MACRO_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * \brief Macro recursion
 *
 * \param[in] macro Macro to be repeated recursively
 * \param[in] arg A recursive threshold, building on this to decline by times
 *                defined with parameter n
 * \param[in] n The number of repetitious calls to macro
 */
#define RECURSION_MACRO(macro, arg, n) RECURSION_MACRO_I(macro, arg, n)

/*
 * \brief Second level is needed to get integer literal from "n" if it is
 *        defined as macro
 */
#define RECURSION_MACRO_I(macro, arg, n) RECURSION##n(macro, arg)

#define RECURSION0(macro, arg)
#define RECURSION1(macro, arg) RECURSION0(macro, DEC_VALUE(arg)) macro(arg, 0)
#define RECURSION2(macro, arg) RECURSION1(macro, DEC_VALUE(arg)) macro(arg, 1)
#define RECURSION3(macro, arg) RECURSION2(macro, DEC_VALUE(arg)) macro(arg, 2)
#define RECURSION4(macro, arg) RECURSION3(macro, DEC_VALUE(arg)) macro(arg, 3)
#define RECURSION5(macro, arg) RECURSION4(macro, DEC_VALUE(arg)) macro(arg, 4)

#ifdef __cplusplus
}
#endif

#include <utils_decrement_macro.h>
#endif /* _UTILS_RECURSION_MACRO_H */
