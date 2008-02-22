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

#include <mwmr/mwmr.h>
#include <soclib/mwmr_controller.h>
#include <hexo/endian.h>
#include "srl_log.h"

static inline void srl_mwmr_read( srl_mwmr_t mwmr, void *buffer, size_t size )
{
	mwmr_read( mwmr, buffer, size );
}

static inline void srl_mwmr_write( srl_mwmr_t mwmr, const void *buffer, size_t size )
{
	mwmr_write( mwmr, buffer, size );
}

static inline size_t srl_mwmr_try_read( srl_mwmr_t mwmr, void *buffer, size_t size )
{
	return mwmr_try_read( mwmr, buffer, size );
}

static inline size_t srl_mwmr_try_write( srl_mwmr_t mwmr, const void *buffer, size_t size )
{
	return mwmr_try_write( mwmr, buffer, size );
}

static inline void srl_mwmr_hw_init( void *coproc, enum SoclibMwmrWay way,
									 size_t no, const srl_mwmr_t mwmr )
{
	mwmr_hw_init(coproc, way, no, mwmr);
}

static inline uint32_t srl_mwmr_status( void *coproc, size_t no )
{
	uint32_t *c = coproc;
	srl_assert(no < MWMR_IOREG_MAX);
	return endian_le32(c[no]);
}

static inline void srl_mwmr_config( void *coproc, size_t no, uint32_t value )
{
	uint32_t *c = coproc;
	srl_assert(no < MWMR_IOREG_MAX);
	c[no] = endian_le32(value);
}

#endif
