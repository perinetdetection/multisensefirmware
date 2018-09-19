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

#ifndef _STDIO_IO_H_INCLUDED
#define _STDIO_IO_H_INCLUDED

#include <hal_io.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 *  \brief Initialize STDIO access
 *  \param[in] io Pointer to IO descriptor,
 *                NULL to discard R/W without any error.
 */
void stdio_io_init(struct io_descriptor *io);

/**
 *  \brief Change IO descriptor for terminal to R/W data
 *  \param[in] io Pointer to IO descriptor,
 *                NULL to discard R/W without any error.
 */
void stdio_io_set_io(struct io_descriptor *io);

/**
 *  \brief Read through specified terminal
 *  \param[out] buf Pointer to buffer to place read data
 *  \param[in] len Data length in number of bytes
 *  \return status
 *  \retval >=0 number of bytes read
 *  \retval <0 error
 */
int32_t stdio_io_read(uint8_t *buf, const int32_t len);

/**
 *  \brief Write through specified terminal
 *  \param[in] buf Pointer to buffer to place data to write
 *  \param[in] len Data length in number of bytes
 *  \return status
 *  \retval >=0 number of bytes read
 *  \retval <0 error
 */
int32_t stdio_io_write(const uint8_t *buf, const int32_t len);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _STDIO_IO_H_INCLUDED */
