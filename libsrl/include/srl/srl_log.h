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

#ifdef CONFIG_MUTEK_CONSOLE

#include <hexo/lock.h>
#include <stdio.h>

#ifndef SRL_VERBOSITY
#define SRL_VERBOSITY VERB_NONE
#endif

enum __srl_verbosity {
    VERB_NONE,
    VERB_TRACE,
    VERB_DEBUG,
    VERB_MAX,
};

extern CONTEXT_LOCAL FILE *context_tty;
extern CPU_LOCAL FILE *cpu_tty;

#define srl_log( l, c ) do {										   \
		if (VERB_##l <= SRL_VERBOSITY) {							   \
			fputs( c, CONTEXT_LOCAL_GET(context_tty) );									   \
		}															   \
	} while (0)

#define srl_log_printf( l, c... ) do {								   \
		if (VERB_##l <= SRL_VERBOSITY) {							   \
			fprintf( CONTEXT_LOCAL_GET(context_tty), c );									   \
		}															   \
	} while (0)

#define srl_assert(expr)                                           \
    do {                                                            \
        if ( ! (expr) ) {                                           \
            srl_log_printf( NONE, "assertion (%s) failed on %s:%d !\n",  \
                             #expr, __FILE__, __LINE__ );           \
            exit(2);                                                \
        }                                                           \
    } while(0)

#define cpu_printf( c... ) do {					\
		fprintf( CPU_LOCAL_GET(cpu_tty), c );	\
	} while (0)


#else /* CONFIG_MUTEK_CONSOLE */

#warning No SRL log output

#define srl_log( l, c ) do {} while(0)
#define srl_log_printf( l, c... ) do {} while(0)

#define srl_assert(expr) \
	do {				 \
		if ( ! (expr) )	 \
			while(1);	 \
	} while (0)

#endif /* CONFIG_MUTEK_CONSOLE */

#endif
