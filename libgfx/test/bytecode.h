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

#include <stdarg.h>
#include <assert.h>
#include <inttypes.h>
#include <stdint.h>
#include <stddef.h>

typedef uint16_t bc_opcode_t;
typedef int8_t bc_error_t;
typedef int8_t bc_bool_t;

#define CONFIG_MUTEK_BYTECODE_SANDBOX
#define CONFIG_MUTEK_BYTECODE_DEBUG
#define CONFIG_MUTEK_BYTECODE_TRACE
#define CONFIG_MUTEK_BYTECODE_VM

#define BC_ALWAYS_INLINE static inline __attribute__((always_inline))

#if __UINTPTR_MAX__ <= 0xffffffff
typedef uint32_t bc_reg_t;
typedef int32_t bc_sreg_t;
# define BC_REG_FORMAT "08" PRIx32
# define INT_PTR_SIZE 32
#else
typedef uint64_t bc_reg_t;
typedef int64_t bc_sreg_t;
# define BC_REG_FORMAT "016" PRIx64
# define INT_PTR_SIZE 64
#endif

/** @internal */
enum bc_flags_s
{
  BC_FLAGS_NATIVE   = 0x01000000,
  BC_FLAGS_SANDBOX  = 0x02000000,
  BC_FLAGS_SIZEMASK = 0x00ffffff,
};

struct bc_context_s;

/** @This can be used to declare bytecode entry points. @see bc_set_pc */
typedef struct bytecode_entry_s bytecode_entry_t;

/** @This specifes status codes returned by the @ref bc_run function.
    @see #BC_STATUS_CUSTOM */
enum bc_run_status_e
{
  BC_RUN_STATUS_END = 0,
  BC_RUN_STATUS_CYCLES = 1,
  BC_RUN_STATUS_BREAK = 2,
  BC_RUN_STATUS_FAULT = 3,
};

/** @This tests if the return status of @ref bc_run is a custom opcode
    or a value specified in @ref bc_run_status_e */
#define BC_STATUS_CUSTOM(op) ((op) & 0x8000)

/** @internal */
typedef bc_opcode_t (bc_run_t)(struct bc_context_s *ctx);

/** @This is the bytecode descriptor header */
struct bc_descriptor_s
{
  const void *code;
  bc_run_t *run;
  uint32_t flags;
};

/** @This defines the virtual machine context.
    @internalcontent */
struct bc_context_s
{
  bc_reg_t v[16];
  union {
    /** Bytecode resume execution pointer. For native code, this is a
        pointer to the machine code instruction. For vm bytecode, this
        is a 16 bits aligned pointer to the next instruction word with
        the bit 0 indicating if the next instruction must be skipped
        on resume. */
    uintptr_t pc;
    const void *vpc;
  };

  const struct bc_descriptor_s *desc;
  uint8_t mode;

  /** address of writable data segment when sandboxed */
  uintptr_t data_base;
  /** mask address of writable data segment when sandboxed */
  uintptr_t data_addr_mask;
  /** @see bc_init_sandbox */
  bc_bool_t sandbox;
  /** maximum number of executed cycles by a single call to @ref bc_run_vm */
  uint16_t max_cycles;
  bc_bool_t trace;
  bc_bool_t trace_regs;
#if CONFIG_MUTEK_BYTECODE_BREAKPOINTS > 0
  uintptr_t bp_list[CONFIG_MUTEK_BYTECODE_BREAKPOINTS];
  uint16_t bp_mask;
  bool_t BITFIELD(bp_skip,1);
#endif
};

/** @This initializes the virtual machine. The initial value of the
    registers is undefined. */
void
bc_init(struct bc_context_s *ctx,
        const struct bc_descriptor_s *desc);

/** @This initializes the virtual machine in sandbox mode. When
    working in sandbox mode, address are translated and the following
    checks are performed:

    @list
      @item Execution of instructions are not allowed outside of the
        code segment specified in the bytecode descriptor. Code base
        address inside the virtual machine is 0.
      @item Load and store instructions addresses are translated from
        0x80000000 to the @tt data_base address and the address is
        masked according to @tt data_addr_bits. Loads
        below 0x8000000 are translated to the code segment.
      @item The @tt ccall instruction can not be used.
      @item The @tt abort instruction is equivalent to @tt die.
    @end list

    When the @tt data_addr_bits parameter is not 0, it must be at
    least 8 and @tt data_base must point to a 8 bytes aligned buffer.

    When in sandbox mode on a 64 bits target, instructions wont touch
    registers above bit 31. This makes the sandbox a 32 bits virtual
    machine.

    The @tt max_cycles parameter specifies the maximum number of
    executed cycles by a single call to @ref bc_run_vm
*/
void bc_init_sandbox(struct bc_context_s *ctx, const struct bc_descriptor_s *desc,
                     void *data_base, uint_fast8_t data_addr_bits,
                     uint_fast32_t max_cycles);

void *
bc_translate_addr(struct bc_context_s *ctx,
                  bc_reg_t addr_, size_t size,
                  bc_bool_t writable);

/** @This updates the remaining number of cycles before the @ref bc_run
    function returns @ref BC_RUN_STATUS_CYCLES. */
BC_ALWAYS_INLINE void
bc_set_cycles(struct bc_context_s *ctx, uint_fast16_t cycles)
{
  assert(ctx->sandbox);
  ctx->max_cycles = cycles;
}

/** @see bc_set_cycles */
BC_ALWAYS_INLINE uint_fast16_t
bc_get_cycles(const struct bc_context_s *ctx)
{
  assert(ctx->sandbox);
  return ctx->max_cycles;
}

/** @This changes the program counter of a sandboxed virtual machine. */
bc_error_t bc_set_sandbox_pc(struct bc_context_s *ctx, uint32_t pc);

/** @This returns the program counter of a sandboxed virtual machine. */
BC_ALWAYS_INLINE uint32_t bc_get_sandbox_pc(const struct bc_context_s *ctx)
{
  assert(ctx->sandbox);
  return ctx->vpc - ctx->desc->code;
}

/** @This initializes a bytecode descriptor from a bytecode loadable
    blob. The format of the blob is:
    @list
      @item flags in 16 bits little endian representation
      @item words count in 16 bits little endian representation
      @item instruction words
    @end list
    The @tt blob pointer must be 16 bits aligned.
*/
bc_error_t
bc_load(struct bc_descriptor_s *desc,
        const uint8_t *blob, size_t len);


/** @see bc_set_regs */
void
bc_set_regs_va(struct bc_context_s *ctx, uint16_t mask, va_list ap);

/** @This set the value of multiple registers of the virtual
    machine. The @tt mask parameter specifies which register must be
    initialized. An additional value of type @ref uintptr_t must be
    passed for each bit set in @tt mask. */
void
bc_set_regs(struct bc_context_s *ctx, uint16_t mask, ...);

/** @This returns the value of one of the 16 virtual machine registers */
BC_ALWAYS_INLINE uintptr_t
bc_get_reg(struct bc_context_s *ctx, uint_fast8_t i)
{
  return ctx->v[i];
}

/** @This returns a pointer to a packed array of bytes stored in virtual
    machine register storage. See the @tt pack and @tt unpack instructions. */
BC_ALWAYS_INLINE uint8_t *
bc_get_bytepack(struct bc_context_s *ctx, uint_fast8_t i)
{
  return (uint8_t*)(ctx->v + i);
}

/** @This returns a pointer to a packed array of bytes stored in
    virtual machine register storage. When the index of the register
    is to high for the specified number of bytes, a pointer to
    register 0 is retured instead. */
BC_ALWAYS_INLINE uint8_t *
bc_get_bytepack_safe(struct bc_context_s *ctx, uint_fast8_t i,
                     size_t bytes)
{
  size_t reg_count = (((bytes - 1) | 3) + 1) >> 2;
  int32_t m = i + reg_count - 17;
  i &= m >> 31;
  return (uint8_t*)(ctx->v + i);
}

/** @This sets the value of one of the 16 virtual machine registers */
BC_ALWAYS_INLINE void
bc_set_reg(struct bc_context_s *ctx, uint_fast8_t i, uintptr_t value)
{
  ctx->v[i] = value;
}

/** @This returns the value of one of the 16 virtual machine registers */
BC_ALWAYS_INLINE const void *
bc_get_pc(struct bc_context_s *ctx)
{
  return ctx->vpc;
}

/** @This sets the value of the virtual machine pc */
BC_ALWAYS_INLINE void
bc_set_pc(struct bc_context_s *ctx, const void *pc)
{
  ctx->vpc = pc;
}

/** @This function enables or disable the bytecode execution trace
    debug output. If the @ref #CONFIG_MUTEK_BYTECODE_TRACE token is
    not defined, this function has no effect. The @tt trace
    instruction can be used to enable and disable trace output. */
BC_ALWAYS_INLINE void
bc_set_trace(struct bc_context_s *ctx, bc_bool_t enabled, bc_bool_t regs)
{
  ctx->trace = enabled;
  ctx->trace_regs = regs;
}

/** @This skip the next instruction. This can only be called if the
    execution has stopped on a conditional custom instruction. */
BC_ALWAYS_INLINE void
bc_skip(struct bc_context_s *ctx)
{
  ctx->pc |= 1;
}

/** @This returns the current bytecode execution mode */
BC_ALWAYS_INLINE uint_fast8_t bc_get_mode(const struct bc_context_s *ctx)
{
  return ctx->mode;
}

/** @This sets the current bytecode execution mode */
BC_ALWAYS_INLINE void bc_set_mode(struct bc_context_s *ctx, uint_fast8_t mode)
{
  ctx->mode = mode & 63;
}

/** @This dumps the virtual machine state. If the @ref
    #CONFIG_MUTEK_BYTECODE_DEBUG token is not defined, this
    function has no effect. */
void bc_dump(const struct bc_context_s *ctx, bc_bool_t regs);

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
BC_ALWAYS_INLINE bc_opcode_t bc_run(struct bc_context_s *ctx)
{
  return ctx->desc->run(ctx);
}

/** This function starts or resumes executions of the bytecode using
    the virtual machine. This function does not work if the bytecode
    is compiled in machine code.

    When sandboxed, at most @tt max_cycles instructions are
    executed. This function returns 1 when this limit is reached. It
    will return 3 if an error occurs. */
bc_opcode_t bc_run_vm(struct bc_context_s *ctx);

/** @internal @This specifies packing and byteswap opcode operations */
enum bc_opcode_pack_e
{
  BC_OP_PACK8       = 0,
  BC_OP_PACK16LE    = 1,
  BC_OP_PACK16BE    = 2,
  BC_OP_UNPACK16LE  = 3,
  BC_OP_UNPACK16BE  = 4,
  BC_OP_SWAP16LE    = 5,
  BC_OP_SWAP16BE    = 6,
  BC_OP_SWAP16      = 7,
  BC_OP_UNPACK8     = 8,
  BC_OP_PACK32LE    = 9,
  BC_OP_PACK32BE    = 10,
  BC_OP_UNPACK32LE  = 11,
  BC_OP_UNPACK32BE  = 12,
  BC_OP_SWAP32LE    = 13,
  BC_OP_SWAP32BE    = 14,
  BC_OP_SWAP32      = 15,
};

/** @see #BC_CCALL_FUNCTION */
#define BC_CCALL_FUNCTION(n) void (n)(struct bc_context_s *ctx)
/** C function type invoked by the @tt ccall instruction. */
typedef BC_CCALL_FUNCTION(bc_ccall_function_t);

#endif

