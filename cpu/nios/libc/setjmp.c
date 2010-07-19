/*
   ; * C library -- _setjmp, _longjmp
   ; *
   ; *      _longjmp(a,v)
   ; * will generate a "return(v?v:1)" from
   ; * the last call to
   ; *      _setjmp(a)
   ; * by unwinding the call stack.
   ; * The previous signal state is NOT restored.
   ; */

#include <setjmp.h>

reg_t setjmp(jmp_buf env);
asm(
    ".set noat				\n"
    ".globl	setjmp			\n"

    "setjmp:				\n"
    "stw	r16, 0(r4)		\n"
    "stw	r17, 4(r4)		\n"
    "stw	r18, 8(r4)		\n"
    "stw	r19, 12(r4)		\n"
    "stw	r20, 16(r4)		\n"
    "stw	r21, 20(r4)		\n"
    "stw	r22, 24(r4)		\n"
    "stw	r23, 28(r4)		\n"
    "stw	gp, 32(r4)		\n"
    "stw	sp, 36(r4)		\n"
    "stw	fp, 40(r4)		\n"
    "stw	ra, 44(r4)		\n"
    "mov	r2, zero		\n"
    "jmp	ra			\n"
    );

void longjmp(jmp_buf env, reg_t val);
asm(
    ".set noat				\n"
    ".globl	longjmp			\n"

    "longjmp:				\n"
    "ldw	r16, 0(r4)		\n"
    "ldw	r17, 4(r4)		\n"
    "ldw	r18, 8(r4)		\n"
    "ldw	r19, 12(r4)		\n"
    "ldw	r20, 16(r4)		\n"
    "ldw	r21, 20(r4)		\n"
    "ldw	r22, 24(r4)		\n"
    "ldw	r23, 28(r4)		\n"
    "ldw	gp, 32(r4)		\n"
    "ldw	sp, 36(r4)		\n"
    "ldw	fp, 40(r4)		\n"
    "ldw	ra, 44(r4)		\n"
    "mov	r2, r5			\n"
    "bne	r2, zero, 1f		\n"
    "movi	r2, 1			\n"
    "1:	jmp	ra			\n"
    );
