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

#ifndef SRL_VERBOSITY
#define SRL_VERBOSITY VERB_NONE
#endif

enum __srl_verbosity {
    VERB_NONE,
    VERB_TRACE,
    VERB_DEBUG,
    VERB_MAX,
};

#if defined(CONFIG_SRL_SOCLIB)
void _srl_log(const char *);
void _srl_log_printf(const char *, ...);
void _cpu_printf(const char *, ...);
#else
# include <stdio.h>

# define _srl_log(x) printf("%s", x)
# define _srl_log_printf(x...) printf(x)
# define _cpu_printf(x...) printf(x)
# define srl_console_init_task(x...)
# define srl_console_init_cpu(x...)
# define srl_console_init(x...)
#endif

#define srl_log( l, c ) do {										   \
		if (VERB_##l <= SRL_VERBOSITY) {							   \
			_srl_log( c );											   \
		}															   \
	} while (0)

#define srl_log_printf( l, c... ) do {								   \
		if (VERB_##l <= SRL_VERBOSITY) {							   \
			_srl_log_printf( c );									   \
		}															   \
	} while (0)

#define cpu_printf( c... ) do {					\
		_cpu_printf( c );						\
	} while (0)

#define srl_assert(expr)												\
    do {																\
        if ( ! (expr) ) {												\
            srl_log_printf(NONE, "assertion (%s) failed on %s:%d !\n",  \
						   #expr, __FILE__, __LINE__ );					\
            abort();													\
        }																\
    } while(0)

#endif
