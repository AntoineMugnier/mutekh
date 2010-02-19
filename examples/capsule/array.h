
#ifndef ARRAY_H_
#define ARRAY_H_

#include <stdint.h>

typedef uint32_t elem_t;

/*
  Array tools
 */

struct array_s {
    size_t size;
    elem_t sum;
    elem_t *array;
};

error_t array_check_sum(struct array_s *array);

ssize_t array_first_unordered(struct array_s *array);

void array_copy(struct array_s *dst, const struct array_s *src);

bool_t array_cmp(const struct array_s *a, const struct array_s *b);

void array_create(struct array_s *array, size_t size, elem_t seed);

void array_dump(struct array_s * array, size_t min, size_t max);

#endif
