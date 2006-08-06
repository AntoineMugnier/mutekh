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

#define	__CONTAINER_RING_TYPE_DECL(name, type, lockname, size) \
							       \
struct				name##_ring_s		       \
{							       \
  __cont_##lockname##_type_t	lock;			       \
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

#define	__CONTAINER_RING_SIZEOF(name) (sizeof ((name##_cont_t*)0)->data \
					 / sizeof(name##_itembase_t))

#define	__CONTAINER_RING_NULL		-1
#define	__CONTAINER_RING_EMPTY		0

#define	__CONTAINER_RING_FUNC(attr, name, prefix, lockname, objprefix, ...) \
									    \
attr __CONTAINER_PROTO_ISNULL(name, prefix)				    \
{									    \
  return index == __CONTAINER_RING_NULL;				    \
}									    \
									    \
attr __CONTAINER_PROTO_GET(name, prefix)				    \
{									    \
  name##_item_t		item;						    \
									    \
  __cont_##lockname##_rdlock(&root->lock);				    \
									    \
  item = root->data[(root->first + index) % __CONTAINER_RING_SIZEOF(name)]; \
									    \
  __cont_##lockname##_unlock(&root->lock);				    \
									    \
  return item;								    \
}									    \
									    \
attr __CONTAINER_PROTO_SET(name, prefix)				    \
{									    \
  uintptr_t	i;							    \
									    \
  __cont_##lockname##_wrlock(&root->lock);				    \
									    \
  i = (root->first + index) % __CONTAINER_RING_SIZEOF(name);		    \
  root->data[i] = item;							    \
									    \
  __cont_##lockname##_unlock(&root->lock);				    \
}									    \
									    \
attr __CONTAINER_PROTO_NEXT(name, prefix)				    \
{									    \
  name##_index_t		res;					    \
									    \
  __cont_##lockname##_rdlock(&root->lock);				    \
									    \
  res = index < root->count - 1						    \
    ? (uintptr_t)(index + 1) % __CONTAINER_RING_SIZEOF(name)		    \
    : __CONTAINER_RING_NULL;						    \
									    \
  __cont_##lockname##_unlock(&root->lock);				    \
									    \
  return res;								    \
}									    \
									    \
attr __CONTAINER_PROTO_PREV(name, prefix)				    \
{									    \
  name##_index_t		res;					    \
									    \
  __cont_##lockname##_rdlock(&root->lock);				    \
									    \
  res = index > 0							    \
    ? (uintptr_t)(index - 1) % __CONTAINER_RING_SIZEOF(name)		    \
    : __CONTAINER_RING_NULL;						    \
									    \
  __cont_##lockname##_unlock(&root->lock);				    \
									    \
  return res;								    \
}									    \
									    \
attr __CONTAINER_PROTO_HEAD(name, prefix)				    \
{									    \
  name##_index_t		res;					    \
									    \
  __cont_##lockname##_rdlock(&root->lock);				    \
									    \
  res = root->count							    \
    ? 0									    \
    : __CONTAINER_RING_NULL;						    \
									    \
  __cont_##lockname##_unlock(&root->lock);				    \
									    \
  return res;								    \
}									    \
									    \
attr __CONTAINER_PROTO_TAIL(name, prefix)				    \
{									    \
  name##_index_t		res;					    \
									    \
  __cont_##lockname##_rdlock(&root->lock);				    \
									    \
  res = root->count							    \
    ? (root->count - 1) % __CONTAINER_RING_SIZEOF(name)			    \
    : __CONTAINER_RING_NULL;						    \
									    \
  __cont_##lockname##_unlock(&root->lock);				    \
									    \
  return res;								    \
}									    \
									    \
attr __CONTAINER_PROTO_COUNT(name, prefix)				    \
{									    \
  size_t	res;							    \
									    \
  __cont_##lockname##_rdlock(&root->lock);				    \
									    \
  res = root->count;							    \
									    \
  __cont_##lockname##_unlock(&root->lock);				    \
									    \
  return res;								    \
}									    \
									    \
attr __CONTAINER_PROTO_MAXCOUNT(name, prefix)				    \
{									    \
  return __CONTAINER_RING_SIZEOF(name);					    \
}									    \
									    \
attr __CONTAINER_PROTO_REMOVE(name, prefix)				    \
{									    \
  uintptr_t	j;							    \
									    \
  __cont_##lockname##_wrlock(&root->lock);				    \
									    \
  for (j = root->first + index;						    \
       j != root->first + root->count - 1; j++)				    \
    root->data[j % __CONTAINER_RING_SIZEOF(name)]			    \
      = root->data[(j + 1) % __CONTAINER_RING_SIZEOF(name)];		    \
									    \
  root->count--;							    \
									    \
  __cont_##lockname##_unlock(&root->lock);				    \
}									    \
									    \
attr __CONTAINER_PROTO_PUSH(name, prefix)				    \
{									    \
  size_t	res = 0;						    \
									    \
  __cont_##lockname##_wrlock(&root->lock);				    \
									    \
  if (root->count < __CONTAINER_RING_SIZEOF(name))			    \
    {									    \
      root->count++;							    \
      root->data[(--root->first)					    \
	         % __CONTAINER_RING_SIZEOF(name)] = item;		    \
      res++;								    \
    }									    \
									    \
  __cont_##lockname##_unlock(&root->lock);				    \
									    \
  return res;								    \
}									    \
									    \
attr __CONTAINER_PROTO_PUSHBACK(name, prefix)				    \
{									    \
  size_t	res = 0;						    \
									    \
  __cont_##lockname##_wrlock(&root->lock);				    \
									    \
  if (root->count < __CONTAINER_RING_SIZEOF(name))			    \
    {									    \
      root->data[(root->first + root->count++)				    \
	         % __CONTAINER_RING_SIZEOF(name)] = item;		    \
      res++;								    \
    }									    \
									    \
  __cont_##lockname##_unlock(&root->lock);				    \
									    \
  return res;								    \
}									    \
									    \
attr __CONTAINER_PROTO_POP(name, prefix)				    \
{									    \
  name##_item_t	res = __CONTAINER_RING_EMPTY;				    \
									    \
  __cont_##lockname##_wrlock(&root->lock);				    \
									    \
  if (root->count > 0)							    \
    {									    \
      root->count--;							    \
      res = root->data[(root->first++)					    \
		       % __CONTAINER_RING_SIZEOF(name)];		    \
    }									    \
									    \
  __cont_##lockname##_unlock(&root->lock);				    \
									    \
  return res;								    \
}									    \
									    \
attr __CONTAINER_PROTO_POPBACK(name, prefix)				    \
{									    \
  name##_item_t	res = __CONTAINER_RING_EMPTY;				    \
									    \
  __cont_##lockname##_wrlock(&root->lock);				    \
									    \
  if (root->count > 0)							    \
    {									    \
      res = root->data[(root->first + --root->count)			    \
		       % __CONTAINER_RING_SIZEOF(name)];		    \
    }									    \
									    \
  __cont_##lockname##_unlock(&root->lock);				    \
									    \
  return res;								    \
}									    \
									    \
attr __CONTAINER_PROTO_PUSH_ARRAY(name, prefix)				    \
{									    \
  uintptr_t	i, j;							    \
									    \
  __cont_##lockname##_wrlock(&root->lock);				    \
									    \
  i = __CONTAINER_RING_SIZEOF(name) - root->count;			    \
  i = i > size ? size : i;						    \
									    \
  for (j = root->first;							    \
       j != root->first - i; )						    \
    root->data[--j % __CONTAINER_RING_SIZEOF(name)] = *item++;		    \
									    \
  root->count += i;							    \
  root->first -= i;							    \
									    \
  __cont_##lockname##_unlock(&root->lock);				    \
									    \
  return i;								    \
}									    \
									    \
attr __CONTAINER_PROTO_PUSHBACK_ARRAY(name, prefix)			    \
{									    \
  uintptr_t	i, j;							    \
									    \
  __cont_##lockname##_wrlock(&root->lock);				    \
									    \
  i = __CONTAINER_RING_SIZEOF(name) - root->count;			    \
  i = i > size ? size : i;						    \
									    \
  for (j = root->first + root->count;					    \
       j != root->first + root->count + i; j++)				    \
    root->data[j % __CONTAINER_RING_SIZEOF(name)] = *item++;		    \
									    \
  root->count += i;							    \
									    \
  __cont_##lockname##_unlock(&root->lock);				    \
									    \
  return i;								    \
}									    \
									    \
attr __CONTAINER_PROTO_POP_ARRAY(name, prefix)				    \
{									    \
  uintptr_t	i, j;							    \
									    \
  __cont_##lockname##_wrlock(&root->lock);				    \
									    \
  i = size > root->count ? root->count : size;				    \
									    \
  for (j = root->first;							    \
       j != root->first + i; j++)					    \
    *item++ = root->data[j % __CONTAINER_RING_SIZEOF(name)];		    \
									    \
  root->first += i;							    \
  root->count -= i;							    \
									    \
  __cont_##lockname##_unlock(&root->lock);				    \
									    \
  return i;								    \
}									    \
									    \
attr __CONTAINER_PROTO_POPBACK_ARRAY(name, prefix)			    \
{									    \
  uintptr_t	i, j;							    \
									    \
  __cont_##lockname##_wrlock(&root->lock);				    \
									    \
  i = size > root->count ? root->count : size;				    \
									    \
  for (j = root->first + root->count;					    \
       j != root->first + root->count - i; )				    \
    *item++ = root->data[--j % __CONTAINER_RING_SIZEOF(name)];		    \
									    \
  root->count -= i;							    \
									    \
  __cont_##lockname##_unlock(&root->lock);				    \
									    \
  return i;								    \
}									    \
									    \
attr __CONTAINER_PROTO_FOREACH(name, prefix)				    \
{									    \
  uintptr_t	i;							    \
  error_t	res = 0;						    \
									    \
  __cont_##lockname##_rdlock(&root->lock);				    \
									    \
  for (i = root->first; i != root->first + root->count; i++)		    \
    if ((res = fcn(root->data[i % __CONTAINER_RING_SIZEOF(name)], param)))  \
      break;								    \
									    \
  __cont_##lockname##_unlock(&root->lock);				    \
									    \
  return res;								    \
}									    \
									    \
attr __CONTAINER_PROTO_INIT(name, prefix)				    \
{									    \
  root->first = 0;							    \
  root->count = 0;							    \
									    \
  return __cont_##lockname##_init(&root->lock);				    \
}									    \
									    \
attr __CONTAINER_PROTO_DESTROY(name, prefix)				    \
{									    \
  __cont_##lockname##_destroy(&root->lock);				    \
}

#endif

