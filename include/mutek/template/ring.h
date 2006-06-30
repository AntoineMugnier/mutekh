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

#ifndef __RING_H_
#define __RING_H_

#include "container.h"

#define	__CONTAINER_RING_TYPE_DECL(name, type, size)	       \
							       \
struct				name##_ring_s		       \
{							       \
  uintptr_t			first;			       \
  size_t			count;			       \
  type				data[size];		       \
};							       \
							       \
typedef intptr_t		name##_index_t;		       \
typedef type			name##_itembase_t;	       \
typedef type			name##_item_t;		       \
typedef struct name##_ring_s	name##_cont_t;		       \
typedef type			name##_entry_t;

#define	__CONTAINER_RING_SIZEOF(name) (sizeof ((struct name##_ring_s*)0)->data \
					 / sizeof(name_##_itembase_t))

#define	__CONTAINER_RING_NULL		-1
#define	__CONTAINER_RING_EMPTY		0

#define	__CONTAINER_RING_FUNC(attr, name, prefix, f)				\
										\
attr __CONTAINER_PROTO_ISNULL(name, prefix)					\
{										\
  return index == __CONTAINER_RING_NULL;					\
}										\
										\
attr __CONTAINER_PROTO_GET(name, prefix)					\
{										\
  return root->data[(root->first + index) % __CONTAINER_RING_SIZEOF(name)];	\
}										\
										\
attr __CONTAINER_PROTO_SET(name, prefix)					\
{										\
  uintptr_t	i = (root->first + index) % __CONTAINER_RING_SIZEOF(name);	\
  name##_item_t	old = root->data[i];						\
										\
  root->data[i] = item								\
										\
  return old;									\
}										\
										\
attr __CONTAINER_PROTO_NEXT(name, prefix)					\
{										\
  return index < root->count - 1						\
    ? (uintptr_t)(index + 1) % __CONTAINER_RING_SIZEOF(name)			\
    : __CONTAINER_RING_NULL;							\
}										\
										\
attr __CONTAINER_PROTO_PREV(name, prefix)					\
{										\
  return index > 0								\
    ? (uintptr_t)(index - 1) % __CONTAINER_RING_SIZEOF(name)			\
    : __CONTAINER_RING_NULL;							\
}										\
										\
attr __CONTAINER_PROTO_HEAD(name, prefix)					\
{										\
  return root->count								\
    ? 0										\
    : __CONTAINER_RING_NULL;							\
}										\
										\
attr __CONTAINER_PROTO_TAIL(name, prefix)					\
{										\
  return root->count								\
    ? (root->count - 1) % __CONTAINER_RING_SIZEOF(name)				\
    : __CONTAINER_RING_NULL;							\
}										\
										\
attr __CONTAINER_PROTO_COUNT(name, prefix)					\
{										\
  return root->count;								\
}										\
										\
attr __CONTAINER_PROTO_DELETE(name, prefix)					\
{										\
  name##_item_t	res = root->data[index];					\
  uintptr_t	j;								\
										\
  for (j = root->first + index;							\
       j < root->first + root->count - 1; j++)					\
    root->data[j % __CONTAINER_RING_SIZEOF(name)]				\
      = root->data[(j + 1) % __CONTAINER_RING_SIZEOF(name)];			\
										\
  root->count--;								\
										\
  return res;									\
}										\
										\
attr __CONTAINER_PROTO_PUSH(name, prefix)					\
{										\
  if (root->count >= __CONTAINER_RING_SIZEOF(name))				\
    return 0;									\
										\
  root->data[(--root->first + root->count++)					\
	     % __CONTAINER_RING_SIZEOF(name)] = item;				\
										\
  return 1;									\
}										\
										\
attr __CONTAINER_PROTO_PUSHBACK(name, prefix)					\
{										\
  if (root->count >= __CONTAINER_RING_SIZEOF(name))				\
    return 0;									\
										\
  root->data[(root->first + root->count++)					\
	     % __CONTAINER_RING_SIZEOF(name)] = item;				\
										\
  return 1;									\
}										\
										\
attr __CONTAINER_PROTO_POP(name, prefix)					\
{										\
  if (root->count == 0)								\
    return __CONTAINER_RING_EMPTY;						\
										\
  root->count--;								\
										\
  return root->data[(root->first++)						\
		    % __CONTAINER_RING_SIZEOF(name)];				\
}										\
										\
attr __CONTAINER_PROTO_POPBACK(name, prefix)					\
{										\
  if (root->count == 0)								\
    return __CONTAINER_RING_EMPTY;						\
										\
  return root->data[(root->first + --root->count)				\
		    % __CONTAINER_RING_SIZEOF(name)];				\
}										\
										\
attr __CONTAINER_PROTO_PUSH_ARRAY(name, prefix)					\
{										\
  uintptr_t	i, j;								\
										\
  i = __CONTAINER_RING_SIZEOF(name) - root->count;				\
  i = i > count ? count : i;							\
										\
  for (j = root->first;								\
       j != root->first - i; )							\
    root->data[--j % __CONTAINER_RING_SIZEOF(name)] = *item++;			\
										\
  root->count += i;								\
  root->first -= i;								\
										\
  return i;									\
}										\
										\
attr __CONTAINER_PROTO_PUSHBACK_ARRAY(name, prefix)				\
{										\
  uintptr_t	i, j;								\
										\
  i = __CONTAINER_RING_SIZEOF(name) - root->count;				\
  i = i > count ? count : i;							\
										\
  for (j = root->first + root->count;						\
       j < root->first + root->count + i; j++)					\
    root->data[j % __CONTAINER_RING_SIZEOF(name)] = *item++;			\
										\
  root->count += i;								\
										\
  return i;									\
}										\
										\
attr __CONTAINER_PROTO_POP_ARRAY(name, prefix)					\
{										\
  uintptr_t	i, j;								\
										\
  i = count > root->count ? root->count : count;				\
										\
  for (j = root->first;								\
       j < root->first + i; j++)						\
    *item++ = root->data[j % __CONTAINER_RING_SIZEOF(name)];			\
										\
  root->first += i;								\
  root->count -= i;								\
										\
  return i;									\
}										\
										\
attr __CONTAINER_PROTO_POPBACK_ARRAY(name, prefix)				\
{										\
  uintptr_t	i, j;								\
										\
  i = count > root->count ? root->count : count;				\
										\
  for (j = root->first + root->count;						\
       j != root->first + root->count - i; )					\
    *item++ = root->data[--j % __CONTAINER_RING_SIZEOF(name)];			\
										\
  root->count -= i;								\
										\
  return i;									\
}										\
										\
attr __CONTAINER_PROTO_INIT(name, prefix)					\
{										\
  root->first = 0;								\
  root->count = 0;								\
}										\
										\
attr __CONTAINER_PROTO_DESTROY(name, prefix)					\
{										\
}

#endif

