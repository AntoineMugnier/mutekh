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


/**
   @file

   Generic FIFO Template using fixed size object table. Should * be
   effecient for strings (char objects) and pointers objects.
*/

#ifndef FIFO_H_
#define FIFO_H_

#include "../types.h"

/**
   Declare the new FIFO type with associated name

   @hideinitializer
   @param name_ new FIFO name
   @param type_ type of data object carried in the FIFO
   @param size_ maximum data objects in the FIFO
*/

#define FIFO_TYPE_DECL(name_, type_, size_)					\
struct name_##_fifo_s							\
{									\
  uintptr_t	first;							\
  size_t	count;							\
  type_		data[size_];						\
};									\
typedef type_	name_##_fifo_t

/**
   Shortcut for FIFO C type

   @hideinitializer
   @param name_ FIFO name
   @return FIFO C type
*/

#define FIFO_TYPE(name_) struct name_##_fifo_s

/**
   Declare a new FIFO variable or field;

   @hideinitializer
   @param name_ FIFO name
   @param field_ field or variable name
*/

#define FIFO_DECL(name_, field_) FIFO_TYPE(name_) field_

/**
   Empty FIFO variable initializer

   @hideinitializer
   @return Empty FIFO C initializer
*/

#define FIFO_DECL_INIT { 0 }

/**
   FIFO sizeof

   @hideinitializer
   @return maximum data objects in the FIFO
*/

#define FIFO_SIZEOF(name_) (sizeof ((struct name_##_fifo_s*)0)->data / sizeof(name_##_fifo_t))

/* FIXME : replace for loops with memcpy */

/**
   FIFO functions declaration. Must be used to declare all functions
   related to a given FIFO name. FIFO access functions will be
   declared static inline in the current scope.

   Refer to fifoname_* functions documentation.
   @hideinitializer
 */

#define FIFO_FUNC(name_)						\
									\
static inline void							\
name_##_fifo_init(struct name_##_fifo_s *fifo)				\
{									\
  fifo->first = 0;							\
  fifo->count = 0;							\
}									\
									\
static inline size_t							\
name_##_fifo_push(struct name_##_fifo_s *fifo, name_##_fifo_t item)	\
{									\
  if (fifo->count >= FIFO_SIZEOF(name_))				\
    return 0;								\
									\
  fifo->data[(fifo->first + fifo->count++) % FIFO_SIZEOF(name_)] = item;\
									\
  return 1;								\
}									\
									\
static inline name_##_fifo_t						\
name_##_fifo_pop(struct name_##_fifo_s *fifo)				\
{									\
  if (fifo->count == 0)							\
    return 0;								\
									\
  fifo->count--;							\
									\
  return fifo->data[(fifo->first++) % FIFO_SIZEOF(name_)];		\
}									\
									\
static inline size_t							\
name_##_fifo_pushlist(struct name_##_fifo_s *fifo,			\
		  const name_##_fifo_t *item, size_t count)		\
{									\
  size_t	i, j;							\
									\
  i = FIFO_SIZEOF(name_) - fifo->count;					\
  i = i > count ? count : i;						\
									\
  for (j = 0; j < i; j++)						\
    fifo->data[(fifo->first + fifo->count + j) % FIFO_SIZEOF(name_)]	\
      = item[j];							\
									\
  fifo->count += i;							\
									\
  return i;								\
}									\
									\
static inline size_t							\
name_##_fifo_poplist(struct name_##_fifo_s *fifo,			\
		 name_##_fifo_t *item, size_t count)			\
{									\
  size_t	i, j;							\
									\
  i = count > fifo->count ? fifo->count : count;			\
									\
  for (j = 0; j < i; j++)						\
    item[j] = fifo->data[(fifo->first + j) % FIFO_SIZEOF(name_)];	\
									\
  fifo->first += i;							\
  fifo->count -= i;							\
									\
  return i;								\
}									\

#ifdef __DOXYGEN_ONLY__

/**
   This is a templated function, `fifoname_' prefix will be replaced
   with real fifo name.

   Initialize fifo field or variable. Static variable
   intialization can be done with FIFO_DECL_INIT.

   @param fifo pointer to fifo
*/

void fifoname_fifo_init(fifo_t *fifo);

/**
   This is a templated function, `fifoname_' prefix will be replaced
   with real fifo name.

   Try push one item on fifo.

   @param fifo pointer to fifo
   @param item item to push
   @return pushed item count
*/

size_t fifoname_fifo_push(fifo_t *fifo, item_t item);

/**
   This is a templated function, `fifoname_' prefix will be replaced
   with real fifo name.

   Try to pop one item from fifo.

   @param fifo pointer to fifo
   @return poped item, 0/NULL if empty
 */

item_t fifoname_fifo_pop(fifo_t *fifo);

/**
   This is a templated function, `fifoname_' prefix will be replaced
   with real fifo name.

   Try push items on fifo.

   @param fifo pointer to fifo
   @param item items table
   @param count item count in table
   @return pushed item count
*/

size_t fifoname_fifo_pushlist(fifo_t *fifo, item_t const *item, size_t count);

/**
   This is a templated function, `fifoname_' prefix will be replaced
   with real fifo name.

   Try pop items from fifo.

   @param fifo pointer to fifo
   @param item destination items table
   @param count item count in table
   @return poped item count
*/

size_t fifoname_fifo_poplist(fifo_t *fifo, item_t *item, size_t count);

#endif

#endif

