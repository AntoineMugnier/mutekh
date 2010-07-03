/*
 * This file is part of DSX, development environment for static
 * SoC applications.
 * 
 * This file is distributed under the terms of the GNU General Public
 * License.
 * 
 * Copyright (c) 2006, Nicolas Pouillon, <nipo@ssji.net>
 *     Laboratoire d'informatique de Paris 6 / ASIM, France
 * 
 *  $Id$
 */

#ifndef SRL_MWMR_H_
#define SRL_MWMR_H_

/**
 * @file
 * @module{SRL}
 * @short MWMR channels access
 */

#include <mwmr/mwmr.h>
#include <hexo/endian.h>
#include <assert.h>

/** @see mwmr_read */
static inline void srl_mwmr_read( srl_mwmr_t channel, void *buffer, size_t size )
{
	mwmr_read( channel, buffer, size );
}

/** @see mwmr_write */
static inline void srl_mwmr_write( srl_mwmr_t channel, const void *buffer, size_t size )
{
	mwmr_write( channel, buffer, size );
}

/** @see mwmr_try_read */
static inline size_t srl_mwmr_try_read( srl_mwmr_t channel, void *buffer, size_t size )
{
	return mwmr_try_read( channel, buffer, size );
}

/** @see mwmr_try_write */
static inline size_t srl_mwmr_try_write( srl_mwmr_t channel, const void *buffer, size_t size )
{
	return mwmr_try_write( channel, buffer, size );
}

# if defined(CONFIG_MWMR_SOCLIB)
/**
   @this retrieves a status word from a mwmr controller.

   @param coproc base address of the controller
   @param no number of the queried status register
   @return the current status word in the mwmr controller
 */
static inline uint32_t srl_mwmr_status( void *coproc, size_t no )
{
	uint32_t *c = coproc;
	assert(no < MWMR_IOREG_MAX);
	return endian_le32(c[no]);
}

/**
   @this sets a configuration word in a mwmr controller.

   @param coproc base address of the controller
   @param no number of the accessed configuration register
   @param value the value to write in the said register
 */
static inline void srl_mwmr_config( void *coproc, size_t no, uint32_t value )
{
	uint32_t *c = coproc;
	assert(no < MWMR_IOREG_MAX);
	c[no] = endian_le32(value);
}
# else
static inline uint32_t srl_mwmr_status( void *coproc, size_t no )
{
	return 0;
}

static inline void srl_mwmr_config( void *coproc, size_t no, uint32_t value )
{
}

# endif

#endif
