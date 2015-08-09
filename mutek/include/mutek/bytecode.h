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
 * Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2013
 */

#ifndef MUTEK_BYTECODE_H_
#define MUTEK_BYTECODE_H_

#include <hexo/types.h>
#include <stdarg.h>

/**
   @file
   @module{Mutek}
   @short Generic bytecode

   This module provides a simple and small bytecode virtual machine
   with a customisable instruction set. Bytecode instruction are 16
   bits wide.

   There are 16 registers which are at least 32 bits wide and large
   enough to hold a pointer.

   A set of generic instructions is provided. The MSB of the generic
   instructions is always sero, leaving half the opcode space for
   application specific opcodes.

   See @sourcelink example/bytecode/test.bc for an example application.

   @table 4
    @item instruction         @item operands      @item opcode                   @item  format

    @item end                 @item               @item @tt{0000 0000 0000 0000} @item  0
    @item dump                @item               @item @tt{0000 0000 0000 0001} @item  0
    @item abort               @item               @item @tt{0000 0000 0000 0010} @item  0
    @item nop                 @item               @item @tt{0000 0000 0000 0100} @item  0
    @item trace               @item               @item @tt{0000 0000 0000 10xx} @item  0
    @item add8                @item r, +/-v       @item @tt{0000 vvvv vvvv rrrr} @item  0
    @item cst8                @item r, v          @item @tt{0001 vvvv vvvv rrrr} @item  0
    @item call8               @item r, lbl        @item @tt{0010 llll llll rrrr} @item  0
    @item jmp8                @item lbl           @item @tt{0010 llll llll 0000} @item  0
    @item ret                 @item r             @item @tt{0010 0000 0000 rrrr} @item  0
    @item loop                @item r, lbl        @item @tt{0011 llll llll rrrr} @item  0

    @item eq                  @item r, r          @item @tt{0100 0000 rrrr rrrr} @item  1
    @item eq0                 @item r             @item @tt{0100 0000 rrrr rrrr} @item  1
    @item neq                 @item r, r          @item @tt{0100 0001 rrrr rrrr} @item  1
    @item neq0                @item r             @item @tt{0100 0001 rrrr rrrr} @item  1
    @item lt                  @item r, r          @item @tt{0100 0010 rrrr rrrr} @item  1
    @item ---                 @item r             @item @tt{0100 0010 rrrr rrrr} @item  1
    @item lteq                @item r, r          @item @tt{0100 0011 rrrr rrrr} @item  1
    @item ---                 @item r             @item @tt{0100 0011 rrrr rrrr} @item  1
    @item add                 @item r, r          @item @tt{0100 0100 rrrr rrrr} @item  1
    @item ---                 @item r             @item @tt{0100 0100 rrrr rrrr} @item  1
    @item sub                 @item r, r          @item @tt{0100 0101 rrrr rrrr} @item  1
    @item neg                 @item r             @item @tt{0100 0101 rrrr rrrr} @item  1
    @item ---                 @item r, r          @item @tt{0100 0110 rrrr rrrr} @item  1
    @item mul32               @item r, r          @item @tt{0100 0111 rrrr rrrr} @item  1
    @item or32                @item r, r          @item @tt{0100 1000 rrrr rrrr} @item  1
    @item ---                 @item r             @item @tt{0100 1000 rrrr rrrr} @item  1
    @item xor32               @item r, r          @item @tt{0100 1001 rrrr rrrr} @item  1
    @item and32               @item r, r          @item @tt{0100 1010 rrrr rrrr} @item  1
    @item ---                 @item r             @item @tt{0100 1010 rrrr rrrr} @item  1
    @item ccall               @item r, r          @item @tt{0100 1011 rrrr rrrr} @item  1
    @item shl32               @item r, r          @item @tt{0100 1100 rrrr rrrr} @item  1
    @item shr32               @item r, r          @item @tt{0100 1101 rrrr rrrr} @item  1
    @item andn32              @item r, r          @item @tt{0100 1110 rrrr rrrr} @item  1
    @item not32               @item r             @item @tt{0100 1110 rrrr rrrr} @item  1
    @item mov                 @item r, r          @item @tt{0100 1111 rrrr rrrr} @item  1
    @item msbs32              @item r             @item @tt{0100 1111 rrrr rrrr} @item  1

    @item tst32[c,s]          @item r, bit        @item @tt{0101 00sb bbbb rrrr} @item  2
    @item bit32[c,s]          @item r, bit        @item @tt{0101 01sb bbbb rrrr} @item  2
    @item shi32[l,r]          @item r, bit        @item @tt{0101 10rb bbbb rrrr} @item  2
    @item ext[s,z]            @item r, bit        @item @tt{0101 11zb bbbb rrrr} @item  2

    @item ld[8,16,32,64][i]   @item r, ra         @item @tt{0110 0ssi aaaa rrrr} @item  3
    @item st[8,16,32,64][i]   @item r, ra         @item @tt{0110 1ssi aaaa rrrr} @item  3
    @item st[8,16,32,64]d     @item r, ra         @item @tt{0111 1ss0 aaaa rrrr} @item  3
    @item gaddr               @item r, lbl        @item @tt{0111 0000 0000 rrrr} @item  3
    @item laddr[16,32]        @item r, lbl        @item @tt{0111 0ss0 0000 rrrr} @item  3
    @item jmp32               @item lbl           @item @tt{0111 0000 ---1 0000} @item  3
    @item call32              @item r, lbl        @item @tt{0111 0000 ---1 rrrr} @item  3
    @item cst[16,32,64]       @item r, v, b       @item @tt{0111 0ss0 bbb1 rrrr} @item  3
    @item ld[8,16,32,64]e     @item r, ra, v      @item @tt{0111 0ss1 aaaa rrrr} @item  3
    @item st[8,16,32,64]e     @item r, ra, v      @item @tt{0111 1ss1 aaaa rrrr} @item  3

    @item .data16             @item v             @item                          @item

    @item custom              @item               @item @tt{1--- ---- ---- ----} @item
   @end table

   Instruction details:

   @list
     @item end: Terminate bytecode execution.
     @item dump: Dump registers to debug output. @see #CONFIG_MUTEK_BYTECODE_DEBUG
     @item abort: Terminate bytecode execution and report an error.
     @item trace: Enable or disable debug trace. @see bc_set_trace
     @item add8: Add a signed 8 bits value to a register.
     @item cst8: Set a register to an unsigned 8 bits constant.
     @item jmp: Jump relative.
     @item jmpf: Jump absolute.
     @item jmpl: Jump relative and save the return address in a register.
     @item call: Jump absolute and save the return address in a register.
     @item ret: Return to the address saved in a register.
     @item loop: If the jump target is backward, this instruction decrements the
     register which should not be initially zero and branch if the
     result is not zero. If the jump target is forward, this instruction decrement the
     register if its initial value is not zero. If the register initial
     value is zero, the branch is taken and the register is left
     untouched.
     @item eq: Compare two registers and skip the next instruction if they are not equal.
     @item neq: Compare two registers and skip the next instruction if thay are equal.
     @item eq0: Skip the next instruction if the register is not zero.
     @item neq0: Skip the next instruction if the register is zero.
     @item lt: Compare two registers and skip the next instruction if
     first reg >= second reg.
     @item lteq: Compare two registers and skip the next instruction if
     first reg > second reg.
     @item mov: Copy value between 2 registers.
     @item add: Add the value of the source register to the destination register.
     @item sub: Subtract the value of the source register from the destination register.
     @item neg: Subtract the value from zero.
     @item mul32: Multiply the values of the source register and destination registers.
     @item ccall: Call a C function. The address of the function is in the source
     register. The value of the destination register is passed to the
     function and the return value is stored back in this same register. (bytecode will not be portable)
     @see bc_call_function_t
     @item or32: 32 bits bitwise or.
     @item xor32: 32 bits bitwise exclusive or.
     @item and32: 32 bits bitwise and.
     @item andn32: 32 bits bitwise and with complemented source register.
     @item not32: 32 bits bitwise not.
     @item shl32: 32 bits variable left shift.
     @item shr32: 32 bits variable right shift.
     @item tst32c: Extract a bit from a register and skip the next instruction if the
     bit is not cleared. Bit index is in the range [0,31].
     @item tst32s: Extract a bit from a register and skip the next instruction if the
     bit is not set. Bit index is in the range [0,31].
     @item bit32c: Clear a bit in a register. Bit index is in the range [0,31].
     @item bit32s: Set a bit in a register. Bit index is in the range [0,31].
     @item shil: Left shift a register by a constant amount.
     Amount must be in the range [0,31].
     @item shi32r: Right shift a register by a constant amount.
     Amount must be in the range [0,31].
     @item msbs32: Find the position of the most significant bit set in range [0, 31].
     @item exts: Sign extend a register using the specified sign bit in the range [0,31].
     @item extz: Clear all bits in a register above specified bit in the range [0,31].
     @item ld*: Load a value from a memory address given by a register into an
     other register.
     @item ld*i: Load a value from a memory address given by a register into an
     other register then add the width of the access to the address
     register.
     @item ld*e: Load a value from a memory address given by a register and a 16
     bits signed offset into an other register.
     @item st*: Store a value to a memory address given by a register from an
     other register.
     @item st*i: Store a value to a memory address given by a register from an
     other register then add the width of the access to the address
     register.
     @item st*d: Subtract the width of the access to the address register then
     store a value from an other register to the resulting memory
     address.
     @item st*e: Store a value from a register to a memory address given by an
     other register and a 16 bits signed offset.
     @item cst*: Set a register to an unsigned constant which may
     be shifted by a mulitple of 8 bits.
     @item laddr*: Set a register to the address of a bytecode label.
     @item gaddr: Set a register to the address of a global symbol. (bytecode will not be portable)
   @end list
*/

typedef uint16_t bc_opcode_t;

#if INT_REG_SIZE <= 32
typedef uint32_t bc_reg_t;
typedef int32_t bc_sreg_t;
#else
typedef uint64_t bc_reg_t;
typedef int64_t bc_sreg_t;
#endif

/** @internal */
enum bc_flags_s
{
  BC_FLAGS_NATIVE = 0x0001,
};

struct bc_context_s;

/** @internal */
typedef reg_t (bc_run_t)(struct bc_context_s *ctx, int_fast32_t max_cycles);

/** @This is the bytecode descriptor header */
struct bc_descriptor_s
{
  uint16_t flags;
  uint16_t op_count;
  const void *code;
  bc_run_t *run;
};

/** @This defines the virtual machine context. */
struct bc_context_s
{
  bc_reg_t v[16];
  union {
    const uint16_t *pc;
    const void *vpc;
  };
  bc_reg_t skip;

  const struct bc_descriptor_s *desc;

#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
  uintptr_t min_addr;
  uintptr_t max_addr;
  bool_t sandbox;
#endif
#ifdef CONFIG_MUTEK_BYTECODE_TRACE
  bool_t trace;
  bool_t trace_regs;
#endif
};

/** @see bc_init */
void bc_init_va(struct bc_context_s *ctx,
		const struct bc_descriptor_s *desc,
		uint_fast8_t pcount, va_list ap);

/** @This intializes the virtual machine. Up to 16 parameters of type
    @ref bc_reg_t can be passed in order to initialize the virtual
    machine registers. Other registers are initialized to 0 except r14
    which defaults to -1.
*/
void
bc_init(struct bc_context_s *ctx,
        const struct bc_descriptor_s *desc,
        uint_fast8_t pcount, ...);

/** @This returns the value of one of the 16 virtual machine registers */
ALWAYS_INLINE bc_reg_t
bc_get_reg(struct bc_context_s *ctx, uint_fast8_t i)
{
  return ctx->v[i];
}

/** @This sets the value of one of the 16 virtual machine registers */
ALWAYS_INLINE void
bc_set_reg(struct bc_context_s *ctx, uint_fast8_t i, bc_reg_t value)
{
  ctx->v[i] = value;
}

/** @This returns the value of one of the 16 virtual machine registers */
ALWAYS_INLINE const void *
bc_get_pc(struct bc_context_s *ctx)
{
  return ctx->vpc;
}

/** @This sets the value of the virtual machine pc */
ALWAYS_INLINE void
bc_set_pc(struct bc_context_s *ctx, const void *pc)
{
  ctx->vpc = pc;
}

/** @This function enables or disable the bytecode execution trace
    debug output. If the @ref #CONFIG_MUTEK_BYTECODE_TRACE token is
    not defined, this function has no effect. The @tt trace
    instruction can be used to enable and disable trace output. */
ALWAYS_INLINE void
bc_set_trace(struct bc_context_s *ctx, bool_t enabled, bool_t regs)
{
#ifdef CONFIG_MUTEK_BYTECODE_TRACE
  ctx->trace = enabled;
  ctx->trace_regs = regs;
#endif
}

/** @This skip the next instruction. This can only be called if the
    execution has stopped on a conditional custom instruction. */
ALWAYS_INLINE void
bc_skip(struct bc_context_s *ctx)
{
  ctx->vpc = (uint8_t*)ctx->vpc + ctx->skip;
}

/** @This can be used to reduce the range of memory addresses which
    can be accessed by the bytecode. If the @ref
    #CONFIG_MUTEK_BYTECODE_CHECKING token is not defined, this
    function has no effect. */
ALWAYS_INLINE void
bc_set_addr_range(struct bc_context_s *ctx, uintptr_t min, uintptr_t max)
{
#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
  ctx->min_addr = min;
  ctx->max_addr = max;
#endif
}

/** @This can be used to disallow use of the @t ccall instruction
    in the bytecode. If the @ref #CONFIG_MUTEK_BYTECODE_CHECKING token
    is not defined, this function has no effect. */
ALWAYS_INLINE void
bc_sandbox(struct bc_context_s *ctx, bool_t sandbox)
{
#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
  ctx->sandbox = sandbox;
#endif
}

/** @This dumps the virtual machine state. If the @ref
    #CONFIG_MUTEK_BYTECODE_DEBUG token is not defined, this
    function has no effect. */
void bc_dump(const struct bc_context_s *ctx, bool_t regs);

/** This function starts or resumes executions of the bytecode. It
    stops when an instruction which is not handled is encountered and
    returns its opcode. Instructions words with the most significant
    bit set are custom instructions and must be handled by the caller
    before resuming execution of the bytecode.

    If the end of bytecode instruction has been reached, this function
    returns 0. Other return values less than 32768 indicate an error
    condition.

    This function will eiter run the vm or jump to the machine
    compiled bytecode. The type of bytecode is guessed from the
    descriptor.
*/
ALWAYS_INLINE bc_opcode_t bc_run(struct bc_context_s *ctx,
                                 int_fast32_t max_cycles)
{
  return ctx->desc->run(ctx, max_cycles);
}

/** This function starts or resumes executions of the bytecode using
    the virtual machine. This function does not work if the bytecode
    is compiled in machine code.

    If the execution took more than @tt max_cycles instructions, this
    function returns 1. It may also stop if an error occurs and the
    @ref #CONFIG_MUTEK_BYTECODE_CHECKING token is defined. */
bc_opcode_t bc_run_vm(struct bc_context_s *ctx, int_fast32_t max_cycles);

/** @internal @This specifies opcode values. */
enum bc_opcode_e
{
  BC_OP_ADD8 = 0x00,
  BC_OP_CST8 = 0x10,
  BC_OP_JMP  = 0x20,
  BC_OP_LOOP = 0x30,

  BC_OP_FMT1 = 0x40,
    BC_OP_EQ   = 0x40,
    BC_OP_NEQ  = 0x41,
    BC_OP_LT   = 0x42,
    BC_OP_LTEQ = 0x43,
    BC_OP_ADD  = 0x44,
    BC_OP_SUB  = 0x45,
    BC_OP_RES  = 0x46,
    BC_OP_MUL  = 0x47,
    BC_OP_OR   = 0x48,
    BC_OP_XOR  = 0x49,
    BC_OP_AND  = 0x4a,
    BC_OP_CCALL = 0x4b,
    BC_OP_SHL  = 0x4c,
    BC_OP_SHR  = 0x4d,
    BC_OP_ANDN = 0x4e,
    BC_OP_MOV  = 0x4f,
  BC_OP_FMT2 = 0x50,
    BC_OP_TSTC = 0x50,
    BC_OP_TSTS = 0x52,
    BC_OP_BITC = 0x54,
    BC_OP_BITS = 0x56,
    BC_OP_SHIL = 0x58,
    BC_OP_SHIR = 0x5a,
    BC_OP_EXTZ = 0x5c,
    BC_OP_EXTS = 0x5e,
  BC_OP_FMT3 = 0x60,
    BC_OP_LD   = 0x60,
    BC_OP_LDI  = 0x61,
    BC_OP_ST   = 0x68,
    BC_OP_STI  = 0x69,
    BC_OP_CST  = 0x70,
    BC_OP_CALL = 0x70,
    BC_OP_STD  = 0x78,
    BC_OP_LDE  = 0x71,
    BC_OP_STE  = 0x79,
};

/** @see #BC_CCALL_FUNCTION */
#define BC_CCALL_FUNCTION(n) bc_reg_t (n)(struct bc_context_s *ctx, bc_reg_t dst)
/** C function type invoked by the @tt ccall instruction. */
typedef BC_CCALL_FUNCTION(bc_ccall_function_t);

#endif

