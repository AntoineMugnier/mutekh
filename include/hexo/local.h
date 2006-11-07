/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/


#ifndef LOCAL_H_
#define LOCAL_H_

#include "cpu/hexo/local.h"

/** pointer to cpu local storage itself */
extern CPU_LOCAL void *__cpu_data_base;

/** pointer to context local storage in cpu local storage */
extern CPU_LOCAL void *__cpu_context_data_base;

/** pointer to context local storage itself */
extern CONTEXT_LOCAL void *__context_data_base;

/** cpu architecture local storage type attribute */
#ifdef CONFIG_SMP
#define CPUARCH_LOCAL	__attribute__((section (".cpuarchdata")))
#else
#define CPUARCH_LOCAL
#endif

/** cpu architecture local storage variable assignement */
#define CPUARCH_LOCAL_SET(n, v)  (n) = (v)

/** cpu architecture local storage variable read access */
#define CPUARCH_LOCAL_GET(n)    (n)

/** get address of cpu architecture local object */
#define CPUARCH_LOCAL_ADDR(n)   (&(n))

#endif

