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

#ifdef DOXYGEN
/**
 * Initialize a local continuation.
 *
 * This operation initializes the local continuation, thereby
 * unsetting any previously set continuation state.
 *
 * \hideinitializer
 */
#define LC_INIT(lc)

/**
 * Set a local continuation.
 *
 * The set operation saves the state of the function at the point
 * where the operation is executed. As far as the set operation is
 * concerned, the state of the function does <b>not</b> include the
 * call-stack or local (automatic) variables, but only the program
 * counter and such CPU registers that needs to be saved.
 *
 * \hideinitializer
 */
#define LC_SET(lc)

/**
 * Resume a local continuation.
 *
 * The resume operation resumes a previously set local continuation, thus
 * restoring the state in which the function was when the local
 * continuation was set. If the local continuation has not been
 * previously set, the resume operation does nothing.
 *
 * \hideinitializer
 */
#define LC_RESUME(lc)

/**
 * Mark the end of local continuation usage.
 *
 * The end operation signifies that local continuations should not be
 * used any more in the function. This operation is not needed for
 * most implementations of local continuation, but is required by a
 * few implementations.
 *
 * \hideinitializer
 */
#define LC_END(lc)

/**
 * \var typedef lc_t;
 *
 * The local continuation type.
 *
 * \hideinitializer
 */
#endif /* DOXYGEN */

#ifndef __LC_H__
#define __LC_H__

#ifdef LC_CONF_INCLUDE
#include LC_CONF_INCLUDE
#else
#include "lc-switch.h"
#endif /* LC_CONF_INCLUDE */
#endif /* __LC_H__ */