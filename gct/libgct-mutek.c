/*

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License as
    published by the Free Software Foundation; either version 2 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
    02110-1301 USA

    As a special exception, if other file call functions, use variables
    or types from this file, the file does not by itself cause the
    resulting work to be covered by the GNU General Public License,
    provided that direct and indirect invocations of GCT template
    macros only appear in the source code of components distributed
    under a free software license listed by the Free Software Fundation
    as compatible with the GNU GPL version 2 or later.

    The source code for the present file must still be made available
    in accordance with section (3) of the GNU General Public
    License. This exception does not invalidate any other reasons why
    a work based on this file might be covered by the GNU General
    Public License.

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (C) 2009
*/

#include "config.h"

/* Enable definition of library functions without static/extern/inline attributes */
#define _GCT_LIBRARY_BUILD

/*
 * All functions are actually defined inside gct headers
 */

#include "gct_atomic.h"
#include "gct_lock_hexo_lock.h"
#include "gct_lock_hexo_lock_irq.h"

