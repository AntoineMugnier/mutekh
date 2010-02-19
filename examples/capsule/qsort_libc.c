#include <stdlib.h>
#include "qsort_libc.h"

/*
 Libc implementation
 */

int_fast8_t qsort_cmp(const void * a, const void * b)
{
    const elem_t *t_a = a;
    const elem_t *t_b = b;

    return *t_a - *t_b;
}
	 
void do_libc(struct array_s *array)
{
    qsort(array->array, array->size, sizeof(* array->array), qsort_cmp);
}
