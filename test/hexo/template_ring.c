
#include <hexo/template/cont_ring.h>

#include <assert.h>
#include <string.h>
#include <stdio.h>

CONTAINER_TYPE_DECL(test, RING, char, NOLOCK, 8);
CONTAINER_FUNC(static inline, test, RING, ring, NOLOCK);

static inline void ring_debug_state(test_cont_t *ring)
{
  printf("first %u, count %u, data %P\n",
	 ring->first, ring->count,
	 ring->data, ring_maxcount(ring));
}

static error_t ring_iterator(char c, void *param)
{
  char	**ptr = param;

  **ptr = c & ~32;
  (*ptr)++;

  return 0;
}

error_t template_ring_test()
{
  test_cont_t	ring;
  char		array[16];
  char		*ptr = array;

  puts("Testing hexo/template/cont_ring.h");

  assert(ring_init(&ring) == 0);

  assert(ring_count(&ring) == 0);
  assert(ring_maxcount(&ring) == 8);

  /* simple push pop tests */

  assert(ring_push(&ring, 'b') == 1);
  assert(ring_push(&ring, 'a') == 1);
  assert(ring_push(&ring, 'r') == 1);

  assert(ring_foreach(&ring, ring_iterator, &ptr) == 0);
  assert(ptr == array + 3);
  assert(!memcmp(array, "RAB", 3));

  assert(ring_count(&ring) == 3);
  assert(ring_pop(&ring) == 'r');
  assert(ring_count(&ring) == 2);
  assert(ring_pop(&ring) == 'a');
  assert(ring_count(&ring) == 1);
  assert(ring_pop(&ring) == 'b');
  assert(ring_count(&ring) == 0);

  /* simple pushback popback tests */

  assert(ring_pushback(&ring, 'b') == 1);
  assert(ring_pushback(&ring, 'a') == 1);
  assert(ring_pushback(&ring, 'r') == 1);

  assert(ring_count(&ring) == 3);
  assert(ring_popback(&ring) == 'r');
  assert(ring_count(&ring) == 2);
  assert(ring_popback(&ring) == 'a');
  assert(ring_count(&ring) == 1);
  assert(ring_popback(&ring) == 'b');
  assert(ring_count(&ring) == 0);

  /* mixed push pop pushback popback tests */

  assert(ring_pushback(&ring, 'b') == 1);
  assert(ring_push(&ring, 'a') == 1);
  assert(ring_pushback(&ring, 'r') == 1);
  assert(ring_push(&ring, 'i') == 1);

  assert(ring_pop(&ring) == 'i');
  assert(ring_pop(&ring) == 'a');
  assert(ring_pop(&ring) == 'b');
  assert(ring_pop(&ring) == 'r');

  assert(ring_pushback(&ring, 'b') == 1);
  assert(ring_push(&ring, 'a') == 1);
  assert(ring_pushback(&ring, 'r') == 1);
  assert(ring_push(&ring, 'i') == 1);

  assert(ring_popback(&ring) == 'r');
  assert(ring_popback(&ring) == 'b');
  assert(ring_popback(&ring) == 'a');
  assert(ring_popback(&ring) == 'i');

  assert(ring_count(&ring) == 0);

  /* test empty pop */

  assert(ring_pop(&ring) == 0);
  assert(ring_count(&ring) == 0);

  /* test isnull head tail next prev */

  assert(ring_isnull(ring_head(&ring)));
  assert(ring_isnull(ring_tail(&ring)));
  assert(ring_push(&ring, 'a') == 1);
  assert(!ring_isnull(ring_head(&ring)));
  assert(!ring_isnull(ring_tail(&ring)));
  assert(ring_push(&ring, 'b') == 1);

  assert(ring_head(&ring) == 0);
  assert(ring_tail(&ring) == 1);

  assert(ring_next(&ring, 0) == 1);
  assert(ring_prev(&ring, 1) == 0);
  assert(ring_isnull(ring_prev(&ring, 0)));
  assert(ring_isnull(ring_next(&ring, 1)));

  /* test set get */

  assert(ring_get(&ring, 0) == 'b');
  assert(ring_get(&ring, 1) == 'a');
  ring_set(&ring, 0, 'c');
  assert(ring_get(&ring, 0) == 'c');

  /* test remove */

  assert(ring_pushback(&ring, 'd') == 1);

  ring_remove(&ring, 1);
  assert(ring_pop(&ring) == 'c');
  assert(ring_pop(&ring) == 'd');

  /* test array functions */

  assert(ring_push_array(&ring, "bari", 4) == 4);

  assert(ring_pop(&ring) == 'i');
  assert(ring_pop(&ring) == 'r');
  assert(ring_pop(&ring) == 'a');
  assert(ring_pop(&ring) == 'b');

  assert(ring_push_array(&ring, "123456789", 9) == 8);
  assert(ring_push(&ring, 'a') == 0);

  assert(ring_pop(&ring) == '8');
  assert(ring_popback(&ring) == '1');

  assert(ring_count(&ring) == 6);

  assert(ring_pop_array(&ring, array, 3) == 3);
  assert(!memcmp(array, "765", 3));

  assert(ring_popback_array(&ring, array, 5) == 3);
  assert(!memcmp(array, "234", 3));

  assert(ring_pushback_array(&ring, "123456789", 9) == 8);
  assert(ring_pop_array(&ring, array, 4) == 4);
  assert(!memcmp(array, "1234", 4));
  assert(ring_pop_array(&ring, array, 6) == 4);
  assert(!memcmp(array, "5678", 4));

  ring_destroy(&ring);

  puts("Test end");

  return 0;
}

