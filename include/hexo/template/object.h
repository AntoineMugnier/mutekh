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

#ifndef __OBJECT_H_
#define __OBJECT_H_

#include <hexo/atomic.h>
#include <assert.h>

#define __OBJECT_PROTO_NEW(name, prefix)			\
name##_object_t							\
prefix##_new	(void *param)

#define __OBJECT_PROTO_DELETE(name, prefix)			\
void								\
prefix##_delete	(name##_object_t object)

#define __OBJECT_PROTO_REFNEW(name, prefix)			\
name##_object_t							\
prefix##_refnew	(name##_object_t object)

#define __OBJECT_PROTO_REFDROP(name, prefix)			\
void								\
prefix##_refdrop(name##_object_t object)

#define OBJECT_TYPE_DECL(name, type)				\
typedef type *			name##_object_t;		\
typedef atomic_t		name##_counter_t;

#define OBJECT_REFCOUNT_INITIALIZER	ATOMIC_INITIALIZER(1)

#define OBJECT_REFCOUNT_FUNC(attr, name, prefix, constructor, destructor, countf)   \
										    \
attr __OBJECT_PROTO_NEW(name, prefix)						    \
{										    \
  name##_object_t		object;						    \
										    \
  if ((object = constructor(param)))						    \
    atomic_set(&object->countf, 1);						    \
										    \
  return object;								    \
}										    \
										    \
attr __OBJECT_PROTO_DELETE(name, prefix)					    \
{										    \
  assert(atomic_get(&object->countf) == 1);					    \
										    \
  destructor(object);  								    \
}										    \
										    \
attr __OBJECT_PROTO_REFNEW(name, prefix)					    \
{										    \
  assert(atomic_get(&object->countf) > 0);					    \
										    \
  atomic_inc(&object->countf);							    \
										    \
  return object;								    \
}										    \
										    \
attr __OBJECT_PROTO_REFDROP(name, prefix)					    \
{										    \
  assert(atomic_get(&object->countf) > 0);					    \
										    \
  if (!atomic_dec(&object->countf))						    \
    {										    \
      destructor(object);  							    \
    }										    \
}

#endif

