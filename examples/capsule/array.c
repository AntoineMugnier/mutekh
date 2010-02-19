#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "array.h"

error_t array_check_sum(struct array_s *array)
{
    size_t i;
    elem_t sum = 0;

    for ( i=0; i<array->size; ++i )
        sum += array->array[i];

    return (array->sum == sum) ? 0 : -EINVAL;
}

ssize_t array_first_unordered(struct array_s *array)
{
    size_t i;

    for ( i=1; i<array->size; ++i )
        if (array->array[i-1] > array->array[i])
            return i-1;

    return -1;
}

void array_copy(struct array_s *dst, const struct array_s *src)
{
    assert(dst->size == src->size);

    memcpy(dst->array, src->array, sizeof(elem_t) * src->size);
    dst->sum = src->sum;
}

bool_t array_cmp(const struct array_s *a, const struct array_s *b)
{
    if (a->size != b->size)
        return 1;

    return memcmp(a->array, b->array, sizeof(elem_t) * a->size);
}

void array_create(struct array_s *array, size_t size, elem_t seed)
{
    size_t i;
    elem_t sum = 0;

    array->array = malloc(sizeof(elem_t) * size);
    array->size = size;

    if ( seed ) {
        srand(seed);
        for ( i=0; i<size; ++i ) {
            elem_t r = rand();
            array->array[i] = r;
            sum += r;
        }

        array->sum = sum;
    }
}

void array_dump(struct array_s * array, size_t min, size_t max)
{
    size_t i;
    if ( !max )
        max = array->size;

    printf("array[%d:%d] =", min, max);
    for ( i=0; i<array->size; ++i ) {
        printf(" ");
        if ( min == i )
            printf("[");
        printf("%d", array->array[i]);
        if ( max-1 == i )
            printf("]");
    }
    printf("\n");
}
