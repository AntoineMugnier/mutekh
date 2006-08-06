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

#include <hexo/template/object.h>
#include <hexo/template/cont_dlist.h>

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

struct test_object_s;

OBJECT_TYPE_DECL(test, struct test_object_s);
CONTAINER_TYPE_DECL(test, DLIST, struct test_object_s, NOLOCK);

struct test_object_s
{
  test_entry_t		list_entry;
  test_counter_t	count;
  char			*data;
};

static test_object_t object_alloc(void *param)
{
  test_object_t	obj;

  if ((obj = malloc(sizeof (*obj))))
    obj->data = param;

  printf("object alloc %p\n", obj);

  return obj;
}

static void object_free(test_object_t obj)
{
  printf("object free %p\n", obj);

  free(obj);
}

OBJECT_REFCOUNT_FUNC(static inline, test, obj, object_alloc, object_free, count);
CONTAINER_OBJECT_FUNC(static inline, test, DLIST, list, NOLOCK, obj, list_entry);

static error_t object_iterator(test_object_t obj, void *param)
{
  printf("object iter %p %s\n", obj, obj->data);

  return 0;
}

error_t template_object_test()
{
  test_cont_t	list;
  test_object_t	obj;

  puts("Testing hexo/template/object.h");

  list_init(&list);

  obj = obj_new("foo");
  list_push(&list, obj);
  obj_refdrop(obj);

  obj = obj_new("bar");
  list_pushback(&list, obj);
  obj_refdrop(obj);

  list_foreach(&list, object_iterator, /* param */ 0);

  obj_refdrop(list_pop(&list));
  obj_refdrop(list_pop(&list));

  puts("Test end");

  return 0;
}

