
#include <setjmp.h>

reg_t setjmp(jmp_buf env);
asm(
        ".set push				\n"
        ".set noat				\n"
        ".globl setjmp          \n"
        "setjmp:                \n"
        /* save gp registers */
        "sw     $16,    16*4($4)\n"
        "sw     $17,    17*4($4)\n"
        "sw     $18,    18*4($4)\n"
        "sw     $19,    19*4($4)\n"
        "sw     $20,    20*4($4)\n"
        "sw     $21,    21*4($4)\n"
        "sw     $22,    22*4($4)\n"
        "sw     $23,    23*4($4)\n"

        "sw     $24,    24*4($4)\n"
        "sw     $25,    25*4($4)\n"

        "sw     $gp,    28*4($4)\n"
        "sw     $sp,    29*4($4)\n"
        "sw     $fp,    30*4($4)\n"
        "sw     $ra,    31*4($4)\n"

        /* return is 0 */
        "move   $2,     $0      \n"
        "jr     $ra             \n"
        ".set pop			    \n"
       );

void longjmp(jmp_buf env, reg_t val);
asm(
        ".set push				\n"
        ".set noat				\n"
        ".globl longjmp         \n"
        "longjmp:               \n"
        /* restore gp registers */
        "lw     $16,    16*4($4)\n"
        "lw     $17,    17*4($4)\n"
        "lw     $18,    18*4($4)\n"
        "lw     $19,    19*4($4)\n"
        "lw     $20,    20*4($4)\n"
        "lw     $21,    21*4($4)\n"
        "lw     $22,    22*4($4)\n"
        "lw     $23,    23*4($4)\n"

        "lw     $24,    24*4($4)\n"
        "lw     $25,    25*4($4)\n"

        "lw     $gp,    28*4($4)\n"
        "lw     $sp,    29*4($4)\n"
        "lw     $fp,    30*4($4)\n"
        "lw     $ra,    31*4($4)\n"

        /* set return value: */
        "move   $2,     $5      \n"
        "bnez   $5,     1f      \n"
        "li     $2,     1       \n"
        "1:                     \n"
        "jr     $ra             \n"
        ".set pop               \n"
    );
