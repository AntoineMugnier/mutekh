/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Institut Telecom / Telecom ParisTech (c) 2011
    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2009-2011
*/

#ifndef SYS_TIME_H_
#define SYS_TIME_H_

#include <hexo/decls.h>

C_HEADER_BEGIN

#include <sys/types.h>

/**
   @file
   @module {Core::C library}
   @short Time-related stuff
 */

struct timeval
{
  time_t          tv_sec;     /* seconds */
  time_usec_t     tv_usec;    /* microseconds */
};

C_HEADER_END

#endif

