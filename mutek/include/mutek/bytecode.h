/*
 * This file is part of MutekH.
 * 
 * MutekH is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; version 2.1 of the License.
 * 
 * MutekH is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with MutekH; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * Alexandre Becoulet <alexandre.becoulet@free.fr>
 */

#ifndef MUTEK_BYTECODE_H_
#define MUTEK_BYTECODE_H_

#include <hexo/types.h>
#include <stdarg.h>

/**
   @file

   @code R
    instruction         params        opcode                  format
 -------------------------------------------------------------------
    end                               0000 0000 0000 0000       0
    dump                              0000 0000 0000 0001       0
    abort                             0000 0000 0000 0002       0
    add8                r, +/-v       0000 vvvv vvvv rrrr       0
    cst8                r, v          0001 vvvv vvvv rrrr       0
    jmp                 lbl           0010 llll llll ----       0
    loop                lbl, r        0011 llll llll rrrr       0

    eq                  r, r          0100 0000 rrrr rrrr       1
    neq                 r, r          0100 0001 rrrr rrrr	1
    ---                 r, r          0100 0010 rrrr rrrr       1
    ---                 r, r          0100 0011 rrrr rrrr	1
    add                 r, r          0100 0100 rrrr rrrr	1
    sub                 r, r          0100 0101 rrrr rrrr	1
    rsb                 r, r          0100 0110 rrrr rrrr	1
    mul                 r, r          0100 0111 rrrr rrrr	1
    or                  r, r          0100 1000 rrrr rrrr	1
    xor                 r, r          0100 1001 rrrr rrrr	1
    and                 r, r          0100 1010 rrrr rrrr	1
    call                r, r          0100 1011 rrrr rrrr	1
    shl                 r, r          0100 1100 rrrr rrrr	1
    shr                 r, r          0100 1101 rrrr rrrr       1
    shra                r, r          0100 1110 rrrr rrrr       1
    mov                 r, r          0100 1111 rrrr rrrr	1

    tst[c,s]            r, bit        0101 0cbb bbbb rrrr       2
    shi[l,r]            r, bit        0101 1lbb bbbb rrrr       2

    ld[8,16,32,64][++]  r, ra         0110 0ssi aaaa rrrr       3
    st[8,16,32,64][++]  r, ra         0110 1ssi aaaa rrrr       3
    cst[16,32,64][c]    r, v          0111 0ssc bbbb rrrr       3
    st[8,16,32,64][--]  r             0111 1ss0 bbbb rrrr       3
    ---                               0111 1--1 ---- ----

    custom                            1--- ---- ---- ----
   @end code

*/

typedef uint16_t bc_opcode_t;

/** @This defines the virtual machine context. */
struct bc_context_s
{
  uintptr_t v[16];
  const bc_opcode_t *code;

#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
  uintptr_t min_addr;
  uintptr_t max_addr;
  uint16_t op_count;
  bool_t allow_call;
#endif
};

/** @see bc_init */
void bc_init_va(struct bc_context_s *ctx,
		const bc_opcode_t *code, uint_fast16_t code_sizeof,
		uint_fast8_t pcount, va_list ap);

/** @This intializes the virtual machine. Up to 16 parameters of type
    @ref uintptr_t can be passed in order to initialize the virtual
    machine registers. Registers 0 to 13 default to 0. Register 14
    defaults to all ones. Register 15 is the program counter and is
    always initialized to 0.

    The @tt code_sizeof parameter is not used if the @ref
    #CONFIG_MUTEK_BYTECODE_CHECKING token is not defined.
*/
void
bc_init(struct bc_context_s *ctx,
        const bc_opcode_t *code, uint_fast16_t code_sizeof,
        uint_fast8_t pcount, ...);

/** @This returns the value of one of the 16 virtual machine registers */
static inline uintptr_t
bc_get_reg(struct bc_context_s *ctx, uint_fast8_t i)
{
  return ctx->v[i];
}

/** @This sets the value of one of the 16 virtual machine registers */
static inline void
bc_set_reg(struct bc_context_s *ctx, uint_fast8_t i, uintptr_t value)
{
  ctx->v[i] = value;
}

/** @This increments the program counter */
static inline void
bc_skip(struct bc_context_s *ctx)
{
  ctx->v[15]++;  
}

/** @This can be used to reduce the range of memory addresses which
    can be accessed by the bytecode. If the @ref
    #CONFIG_MUTEK_BYTECODE_CHECKING token is not defined, this
    function has no effect. */
static inline void
bc_set_addr_range(struct bc_context_s *ctx, uintptr_t min, uintptr_t max)
{
#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
  ctx->min_addr = min;
  ctx->max_addr = max;
#endif
}

/** @This can be used to disallow use of the @ref #BC_CALL instruction
    in the bytecode. If the @ref #CONFIG_MUTEK_BYTECODE_CHECKING token
    is not defined, this function has no effect. */
static inline void
bc_allow_call(struct bc_context_s *ctx, bool_t allow)
{
#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
  ctx->allow_call = allow;
#endif
}

/** @This dumps the virtual machine state. */
void bc_dump(const struct bc_context_s *ctx);

/** This function starts or resumes executions of the bytecode. It
    stops when an instruction which is not handled is encountered and
    returns its opcode. It may also stop if an error occurs and the
    @ref #CONFIG_MUTEK_BYTECODE_CHECKING token is defined.

    If the end of bytecode instruction has been reached, this function
    returns 0. If the execution took more than @tt max_cycles
    instructions, this function returns 1.

    Instructions words with the most significant bit set are custom
    instructions and must be handled by the caller before resuming
    execution of the bytecode.

    Other return values indicate an error condition.

    In any case, the program counter points to the instruction which
    made the execution stop. The @ref bc_skip function can be called
    to advance to the next instruction before resuming.
*/
uint16_t bc_run(struct bc_context_s *ctx, int_fast32_t max_cycles);

/** @This specifies operation codes. */
enum bc_opcode_e
{
  BC_OP_ADD8 = 0x00,
  BC_OP_CST8 = 0x10,
  BC_OP_JMP  = 0x20,
  BC_OP_LOOP = 0x30,

  BC_OP_FMT1 = 0x40,
    BC_OP_EQ   = 0x40,
    BC_OP_NEQ  = 0x41,
    BC_OP_RES1 = 0x42,
    BC_OP_RES2 = 0x43,
    BC_OP_ADD  = 0x44,
    BC_OP_SUB  = 0x45,
    BC_OP_RSB  = 0x46,
    BC_OP_MUL  = 0x47,
    BC_OP_OR   = 0x48,
    BC_OP_XOR  = 0x49,
    BC_OP_AND  = 0x4a,
    BC_OP_CALL = 0x4b,
    BC_OP_SHL  = 0x4c,
    BC_OP_SHR  = 0x4d,
    BC_OP_SHRA = 0x4e,
    BC_OP_MOV  = 0x4f,
  BC_OP_FMT2 = 0x50,
    BC_OP_TSTC = 0x50,
    BC_OP_TSTS = 0x54,
    BC_OP_SHIL = 0x58,
    BC_OP_SHIR = 0x5c,
  BC_OP_FMT3 = 0x60,
    BC_OP_LD   = 0x60,
    BC_OP_ST   = 0x68,
    BC_OP_CST  = 0x70,
    BC_OP_ST2  = 0x78,
};

/** @see #BC_CALL_FUNCTION */
#define BC_CALL_FUNCTION(n) uintptr_t (n)(struct bc_context_s *ctx, uintptr_t dst)
/** C function type invoked by the @ref #BC_CALL instruction. */
typedef BC_CALL_FUNCTION(bc_call_function_t);

/** @multiple @internal */
#define BC_FMT0(op, value, reg) (((op) << 8) | (((value) & 0xff) << 4) | ((reg) & 0xf))
#define BC_FMT1(op, rdst, rsrc) (((op) << 8) | (((rsrc) & 0xf) << 4) | ((rdst) & 0xf))
#define BC_FMT2(op, reg, bit)   (((op) << 8) | (((bit) & 0x3f) << 4) | ((reg) & 0xf))
#define BC_FMT3(op, size, reg, a, b)  (((op) << 8) | (((size) & 3) << 9) | (((b) & 1) << 8) | (((a) & 0xf) << 4) | ((reg) & 0xf))

/** Terminate bytecode execution */
#define BC_END()            BC_FMT0(BC_OP_ADD8, 0, 0)
/** Dump registers to debug output. @see #CONFIG_MUTEK_BYTECODE_DEBUG */
#define BC_DUMP()           BC_FMT0(BC_OP_ADD8, 0, 1)
/** Terminate bytecode execution and report an error */
#define BC_ABORT()          BC_FMT0(BC_OP_ADD8, 0, 2)
/** Add a signed 8 bits value to a register */
#define BC_ADD8(r, v)       BC_FMT0(BC_OP_ADD8, v, r)
/** Set a register to an unsigned 8 bits constant */
#define BC_CST8(r, v)       BC_FMT0(BC_OP_CST8, v, r)

/** Jump relative. Jump offset is an 8 bits signed value.  Negative
    values jump backward, -1 jump to itself.  Some constant loading
    instructions account for more than 1 instruction word. */
#define BC_JMP(d)           BC_FMT0(BC_OP_JMP,  d, 0)
/** If the jump target is backward, this instruction decrements the
    register which should not be initially zero and branch if the
    result is not zero.

    If the jump target is forward, this instruction decrement the
    register if its initial value is not zero. If the register initial
    value is zero, the branch is taken and the register is left
    untouched. @see #BC_JMP */
#define BC_LOOP(r, d)       BC_FMT0(BC_OP_LOOP, d, r)

/** Compare two registers and skip the next instruction if not equal */
#define BC_EQ(r1, r2)         BC_FMT1(BC_OP_EQ, r1, r2)
/** Compare two registers and skip the next instruction if equal */
#define BC_NEQ(r1, r2)        BC_FMT1(BC_OP_NEQ, r1, r2)

/** Copy value between 2 registers */
#define BC_MOV(dst, src)      BC_FMT1(BC_OP_MOV, dst, src)
/** Add the value of the source register to the destination register */
#define BC_ADD(dst, src)      BC_FMT1(BC_OP_ADD, dst, src)
/** Subtract the value of the source register from the destination register */
#define BC_SUB(dst, src)      BC_FMT1(BC_OP_SUB, dst, src)
/** Subtract the value of the destination register from the source register */
#define BC_RSUB(dst, src)     BC_FMT1(BC_OP_RSUB, dst, src)
/** Multiply the values of the source register and destination registers */
#define BC_MUL(dst, src)      BC_FMT1(BC_OP_MUL, dst, src)

/** Call a C function. The address of the function is in the source
    register. The value of the destination register is passed to the
    function and the return value is stored back in this same register. 
    @see bc_call_function_t @see #BC_CALL_FUNCTION
*/
#define BC_CALL(dst, src)       BC_FMT1(BC_OP_CALL, dst, src)

/** Bitwise or */
#define BC_OR(dst, src)       BC_FMT1(BC_OP_OR, dst, src)
/** Bitwise exclusive or */
#define BC_XOR(dst, src)      BC_FMT1(BC_OP_XOR, dst, src)
/** Bitwise and */
#define BC_AND(dst, src)      BC_FMT1(BC_OP_AND, dst, src)
/** Variable left shift  */
#define BC_SHL(dst, src)      BC_FMT1(BC_OP_SHL, dst, src)
/** Variable right shift */
#define BC_SHR(dst, src)      BC_FMT1(BC_OP_SHR, dst, src)
/** Variable arithmetic left shift */
#define BC_SHRA(dst, src)     BC_FMT1(BC_OP_SHRA, dst, src)

/** Extract a bit from a register and skip the next instruction if the
    bit is not cleared. */
#define BC_TSTC(r, bit)       BC_FMT2(BC_OP_TSTC, r, bit)
/** Extract a bit from a register and skip the next instruction if the
    bit is not set. */
#define BC_TSTS(r, bit)       BC_FMT2(BC_OP_TSTS, r, bit)

/** Left shift a register by a constant amount */
#define BC_SHIL(r, bit)       BC_FMT2(BC_OP_SHIL, r, bit)
/** Right shift a register by a constant amount */
#define BC_SHIR(r, bit)       BC_FMT2(BC_OP_SHIR, r, bit)

/** Load a value from a memory address given by a register into an
    other register. @multiple */
#define BC_LD8(r, a)          BC_FMT3(BC_OP_LD, 0, r, a, 0)
#define BC_LD16(r, a)         BC_FMT3(BC_OP_LD, 1, r, a, 0)
#define BC_LD32(r, a)         BC_FMT3(BC_OP_LD, 2, r, a, 0)
#define BC_LD64(r, a)         BC_FMT3(BC_OP_LD, 3, r, a, 0)
/** Load a value from a memory address given by a register into an
    other register then add the width of the access to the address
    register. @multiple */
#define BC_LD8I(r, a)         BC_FMT3(BC_OP_LD, 0, r, a, 1)
#define BC_LD16I(r, a)        BC_FMT3(BC_OP_LD, 1, r, a, 1)
#define BC_LD32I(r, a)        BC_FMT3(BC_OP_LD, 2, r, a, 1)
#define BC_LD64I(r, a)        BC_FMT3(BC_OP_LD, 3, r, a, 1)

/** Store a value to a memory address given by a register from an
    other register. @multiple */
#define BC_ST8(r, a)          BC_FMT3(BC_OP_ST, 0, r, a, 0)
#define BC_ST16(r, a)         BC_FMT3(BC_OP_ST, 1, r, a, 0)
#define BC_ST32(r, a)         BC_FMT3(BC_OP_ST, 2, r, a, 0)
#define BC_ST64(r, a)         BC_FMT3(BC_OP_ST, 3, r, a, 0)

/** Store a value to a memory address given by a register from an
    other register then add the width of the access to the address
    register. @multiple */
#define BC_ST8I(r, a)         BC_FMT3(BC_OP_ST, 0, r, a, 1)
#define BC_ST16I(r, a)        BC_FMT3(BC_OP_ST, 1, r, a, 1)
#define BC_ST32I(r, a)        BC_FMT3(BC_OP_ST, 2, r, a, 1)
#define BC_ST64I(r, a)        BC_FMT3(BC_OP_ST, 3, r, a, 1)

/** Subtract the width of the access to the address register then
    store a value from an other register to the resulting memory
    address. @multiple */
#define BC_ST8D(r, a)         BC_FMT3(BC_OP_ST2, 0, r, a, 0)
#define BC_ST16D(r, a)        BC_FMT3(BC_OP_ST2, 1, r, a, 0)
#define BC_ST32D(r, a)        BC_FMT3(BC_OP_ST2, 2, r, a, 0)
#define BC_ST64D(r, a)        BC_FMT3(BC_OP_ST2, 3, r, a, 0)

/** Load/Store operations with access width matching pointer
    width. @multiple @see {BC_LD32, BC_LD32I, BC_ST32, BC_ST32I, BC_ST32I, BC_ST32D} */
#if INT_PTR_SIZE == 64
# define BC_LDPTR(r, a)        BC_LD64(r, a)
# define BC_LDPTRI(r, a)       BC_LD64I(r, a)
# define BC_STPTR(r, a)        BC_ST64(r, a)
# define BC_STPTRI(r, a)       BC_ST64I(r, a)
# define BC_STPTRD(r, a)       BC_ST64D(r, a)
#elif INT_PTR_SIZE == 32
# define BC_LDPTR(r, a)        BC_LD32(r, a)
# define BC_LDPTRI(r, a)       BC_LD32I(r, a)
# define BC_STPTR(r, a)        BC_ST32(r, a)
# define BC_STPTRI(r, a)       BC_ST32I(r, a)
# define BC_STPTRD(r, a)       BC_ST32D(r, a)
#elif INT_PTR_SIZE == 16
# define BC_LDPTR(r, a)        BC_LD16(r, a)
# define BC_LDPTRI(r, a)       BC_LD16I(r, a)
# define BC_STPTR(r, a)        BC_ST16(r, a)
# define BC_STPTRI(r, a)       BC_ST16I(r, a)
# define BC_STPTRD(r, a)       BC_ST16D(r, a)
#endif

/** Set a register to an unsigned 16 bits constant. This instruction
    is 2 instruction words long. */
#define BC_CST16(r, v)        BC_FMT3(BC_OP_CST, 0, r, 0, 0), ((v) & 0xffff)
/** Set a register to an unsigned 16 bits constant. The constant may
    be shifted by a mulitple of 4 bits and complemented. This
    instruction is 2 instruction words long. */
#define BC_CST16X(r, v, s, c) BC_FMT3(BC_OP_CST, 0, r, (s)/4, c), ((v) & 0xffff)
/** Set a register to an unsigned 32 bits constant. This instruction
    is 3 instruction words long. */
#define BC_CST32(r, v)        BC_FMT3(BC_OP_CST, 1, r, 0, 0), (((v) >> 16) & 0xffff), ((v) & 0xffff)
/** Set a register to an unsigned 32 bits constant. This instruction
    is 5 instruction words long. */
#define BC_CST64(r, v)        BC_FMT3(BC_OP_CST, 3, r, 0, 0), (((v) >> 48) & 0xffff), (((v) >> 32) & 0xffff), (((v) >> 16) & 0xffff), ((v) & 0xffff)

/** Custom bytecode instruction */
#define BC_CUSTOM(op)         ((op) | 0x8000)

#endif

