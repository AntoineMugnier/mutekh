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

#ifndef SRL_LOG_H_
#define SRL_LOG_H_

#include <stdio.h>

enum __srl_verbosity {
    VERB_NONE,
    VERB_TRACE,
    VERB_DEBUG,
    VERB_MAX,
};

#define srl_log( l, c ) do {										   \
		if (VERB_##l >= SRL_VERBOSITY)								   \
			puts( c );												   \
	} while (0)

#define srl_log_printf( l, c... ) do {								   \
		if (VERB_##l >= SRL_VERBOSITY)								   \
			printf( c );											   \
	} while (0)

#define srl_assert(expr)                                           \
    do {                                                            \
        if ( ! (expr) ) {                                           \
            srl_log_printf( NONE, "assertion (%s) failed on %s:%d !\n",  \
                             #expr, __FILE__, __LINE__ );           \
            exit(2);                                                \
        }                                                           \
    } while(0)


#endif
