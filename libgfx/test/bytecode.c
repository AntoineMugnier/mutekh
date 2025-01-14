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

#include "bytecode.h"

#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <stdio.h>

static inline uint16_t load16_le(const uint16_t *x)
{
  const uint8_t *t = (void*)x;
  return t[0] | t[1] << 8;
}

static inline uint16_t endian_swap16(uint16_t x)
{
  return (x >> 8) | (x << 8);
}

static inline uint32_t endian_swap32(uint32_t x)
{
  return (((x >> 24) & 0x000000ff) |
	  ((x >> 8 ) & 0x0000ff00) |
	  ((x << 8 ) & 0x00ff0000) |
	  ((x << 24) & 0xff000000));
}

void
bc_set_regs_va(struct bc_context_s *ctx, uint16_t mask, va_list ap)
{
  while (mask)
    {
      uint_fast8_t r = __builtin_ctz(mask);
      ctx->v[r] = va_arg(ap, uintptr_t);
      mask ^= 1 << r;
    }
}

void
bc_set_regs(struct bc_context_s *ctx, uint16_t mask, ...)
{
  va_list ap;
  va_start(ap, mask);
  bc_set_regs_va(ctx, mask, ap);
  va_end(ap);
}

bc_error_t
bc_desc_init(struct bc_descriptor_s *desc,
             const void *code, size_t len,
             enum bc_flags_s flags)
{
  assert(!((uintptr_t)code & 1));
  desc->run = &bc_run_vm;
  desc->code = code;
  desc->flags = flags | len;
  return 0;
}

bc_error_t
bc_load(struct bc_descriptor_s *desc,
        const uint8_t *blob, size_t len)
{
  if ((uintptr_t)blob & 1)
    return -ENOTSUP;

  if (len < /* header */ 4)
    return -EINVAL;

  uint32_t flags = blob[2] << 16 | blob[3] << 24 |
                   blob[0] | blob[1] << 8;
  size_t bytes = flags & BC_FLAGS_SIZEMASK;

  if (len < /* header */ 4 + bytes)
    return -EINVAL;

  return bc_desc_init(desc, blob + 4, 0, flags);
}

void
bc_init(struct bc_context_s *ctx,
        const struct bc_descriptor_s *desc)
{
  ctx->vpc = desc->code;
  ctx->mode = 0;
  ctx->desc = desc;
  ctx->sandbox = 0;
  ctx->max_cycles = -1;
  ctx->trace = 0;
  ctx->trace_regs = 0;
#if CONFIG_MUTEK_BYTECODE_BREAKPOINTS > 0
  ctx->bp_mask = 0;
  ctx->bp_skip = 0;
#endif
}

void
bc_init_sandbox(struct bc_context_s *ctx, const struct bc_descriptor_s *desc,
                void *data_base, uint_fast8_t data_addr_bits,
                uint_fast32_t max_cycles)
{
  ctx->vpc = desc->code;
  ctx->mode = 0;
  ctx->desc = desc;
  ctx->sandbox = 1;
  ctx->max_cycles = max_cycles;
  if (data_addr_bits)
    {
      assert(data_addr_bits >= 3);
      assert(((uintptr_t)data_base & 7) == 0);
      ctx->data_base = (uintptr_t)data_base;
      ctx->data_addr_mask = (1 << data_addr_bits) - 1;
    }
  else
    {
      static uint64_t dummy;
      ctx->data_base = (uintptr_t)&dummy;
      ctx->data_addr_mask = 7;
    }
  ctx->trace = 0;
  ctx->trace_regs = 0;
#if CONFIG_MUTEK_BYTECODE_BREAKPOINTS > 0
  ctx->bp_mask = 0;
  ctx->bp_skip = 0;
#endif
}

static const char * bc_opname(uint16_t op)
{
  struct op_s
  {
    uint16_t   mask;
    uint16_t   op;
    const char *name, *name1;
  };
  static const struct op_s ops[] =
  {
    { 0x8000, 0x8000, "custom" },
    { 0xffff, 0x0000, "end" },
    { 0xffff, 0x0001, "dump" },
    { 0xffff, 0x0002, "abort" },
    { 0xffff, 0x0003, "die" },
    { 0xfffc, 0x0008, "trace" },
    { 0xfff0, 0x0000, "nop" },
    { 0xf000, 0x0000, "add8" },
    { 0xf000, 0x1000, "cst8" },
    { 0xfff0, 0x2000, "ret" },
    { 0xf00f, 0x2000, "jmp8" },
    { 0xf000, 0x2000, "call8" },
    { 0xf800, 0x3000, "loop" },
    { 0xf800, 0x3800, "(un)pack/swap" },
    { 0xff00, 0x4000, "(n)eq", "eq0" },
    { 0xff00, 0x4100, "mov", "neq0" },
    { 0xff00, 0x4200, "lt", "exts8" },
    { 0xff00, 0x4300, "lts", "exts16" },
    { 0xff00, 0x4400, "lteq", "exts32" },
    { 0xff00, 0x4500, "lteqs" },
    { 0xff00, 0x4600, "add" },
    { 0xff00, 0x4700, "sub", "neg" },
    { 0xff00, 0x4800, "or", "rand" },
    { 0xff00, 0x4900, "xor", "ccall" },
    { 0xff00, 0x4a00, "and" },
    { 0xff00, 0x4b00, "sha", "not" },
    { 0xff00, 0x4c00, "shl" },
    { 0xff00, 0x4d00, "shr" },
    { 0xff00, 0x4e00, "mul" },
    { 0xff00, 0x4f00, "div", "msbs" },
    { 0xfe00, 0x5000, "tstc" },
    { 0xfe00, 0x5200, "tsts" },
    { 0xfe00, 0x5400, "bitc" },
    { 0xfe00, 0x5600, "bits" },
    { 0xfe00, 0x5800, "shil" },
    { 0xfe00, 0x5a00, "shir" },
    { 0xfe00, 0x5c00, "shia" },
    { 0xfe00, 0x5e00, "extz" },
    { 0xf300, 0x6000, "ld" },
    { 0xf300, 0x6100, "ldi" },
    { 0xf300, 0x6100, "st" },
    { 0xf300, 0x6300, "sti" },
    { 0xf3f0, 0x7010, "mode" },
    { 0xf3f0, 0x7000, "gaddr" },
    { 0xf380, 0x7080, "cst" },
    { 0xf3a0, 0x7020, "laddr" },
    { 0xf3f0, 0x7040, "jmp" },
    { 0xf3f0, 0x7050, "call" },
    { 0xf300, 0x7200, "std" },
    { 0xf300, 0x7100, "lde" },
    { 0xf300, 0x7300, "ste" },
    { 0x0000, 0x0000, "invalid" },
  };
  uint_fast8_t i;
  for (i = 0; ; i++)
    if ((op & ops[i].mask) == ops[i].op)
      return !ops[i].name1 || ((op ^ (op >> 4)) & 15) ? ops[i].name : ops[i].name1;
  return NULL;
}

static void bc_dump_op(const struct bc_context_s *ctx, const uint16_t *pc)
{
  const void *code = ctx->desc->code;
  size_t size = ctx->desc->flags & BC_FLAGS_SIZEMASK;

  fprintf(stderr, "bytecode: pc=%p (%u)", pc, (unsigned)((pc - (uint16_t*)code)));

  if (pc >= (uint16_t*)code &&
      (uint8_t*)pc < (uint8_t*)code + size &&
      !((uintptr_t)pc & 1))
    {
      uint16_t op = load16_le(pc);
      fprintf(stderr, ", opcode=%04x (%s)", op, bc_opname(op));
    }

  fprintf(stderr, ", mode=%u\n", ctx->mode);
}

static void bc_dump_regs(const struct bc_context_s *ctx)
{
  uint_fast8_t i;
  for (i = 0; i < 16; i++)
    fprintf(stderr, "r%02u=%" BC_REG_FORMAT "%c", i, ctx->v[i], (i + 1) % 4 ? ' ' : '\n');
}

static void bc_dump_pc(const struct bc_context_s *ctx, const uint16_t *pc)
{
  bc_dump_op(ctx, pc);
  bc_dump_regs(ctx);
}

void bc_dump(const struct bc_context_s *ctx, bc_bool_t regs)
{
  bc_dump_op(ctx, (void*)(ctx->pc & (intptr_t)-2));
  if (regs)
    bc_dump_regs(ctx);
}

#define BC_PACK(n)				\
static void bc_pack##n(void *t, uint_fast8_t c)	\
{						\
  const bc_reg_t *s = t;                        \
  uint##n##_t *d = t;				\
  uint_fast8_t i;				\
  for (i = 0; i < c; i++)			\
    d[i] = s[i];				\
}

#define BC_UNPACK(n)					\
static void bc_unpack##n(void *t, uint_fast8_t c)	\
{							\
  const uint##n##_t *s = t;				\
  bc_reg_t *d = t;                                      \
  uint_fast8_t i;					\
  for (i = c; i--; )					\
    d[i] = s[i];					\
}

BC_PACK(8);
BC_UNPACK(8);
BC_PACK(16);
BC_UNPACK(16);
#if INT_REG_SIZE > 32 || defined(CONFIG_MUTEK_BYTECODE_VM64)
BC_PACK(32);
BC_UNPACK(32);
#else
# define bc_pack32(...)
# define bc_unpack32(...)
#endif

static void bc_swap16(void *t, uint_fast8_t c)
{
  const bc_reg_t *s = t;
  bc_reg_t *d = t;
  uint_fast8_t i;
  for (i = 0; i < c; i++)
    d[i] = endian_swap16(s[i]);
}

static void bc_swap32(void *t, uint_fast8_t c)
{
  const bc_reg_t *s = t;
  bc_reg_t *d = t;
  uint_fast8_t i;
  for (i = 0; i < c; i++)
    d[i] = endian_swap32(s[i]);
}

__attribute__((noinline))
static void bc_run_packing(struct bc_context_s *ctx,
                           uint_fast8_t c, uint_fast8_t r,
                           uint_fast8_t op)
{
  bc_reg_t *t = ctx->v + r;

  switch (op)
    {
    case BC_OP_UNPACK8:
      bc_unpack8(t, c);
      break;
    case BC_OP_UNPACK16LE:
    case BC_OP_UNPACK16BE:
      bc_unpack16(t, c);
      break;
#if INT_REG_SIZE > 32 || defined(CONFIG_MUTEK_BYTECODE_VM64)
    case BC_OP_UNPACK32LE:
    case BC_OP_UNPACK32BE:
      bc_unpack32(t, c);
      break;
#endif
    }

  switch (op)
    {
    case BC_OP_SWAP16:
#if !defined (CONFIG_CPU_ENDIAN_BIG)
    case BC_OP_SWAP16BE:
    case BC_OP_UNPACK16BE:
    case BC_OP_PACK16BE:
#else
    case BC_OP_SWAP16LE:
    case BC_OP_UNPACK16LE:
    case BC_OP_PACK16LE:
#endif
      bc_swap16(t, c);
      break;

    case BC_OP_SWAP32:
#if !defined (CONFIG_CPU_ENDIAN_BIG)
    case BC_OP_SWAP32BE:
    case BC_OP_UNPACK32BE:
    case BC_OP_PACK32BE:
#else
    case BC_OP_SWAP32LE:
    case BC_OP_UNPACK32LE:
    case BC_OP_PACK32LE:
#endif
      bc_swap32(t, c);
      break;
    }

  switch (op)
    {
    case BC_OP_PACK8:
      bc_pack8(t, c);
      break;
    case BC_OP_PACK16LE:
    case BC_OP_PACK16BE:
      bc_pack16(t, c);
      break;
#if INT_REG_SIZE > 32 || defined(CONFIG_MUTEK_BYTECODE_VM64)
    case BC_OP_PACK32LE:
    case BC_OP_PACK32BE:
      bc_pack32(t, c);
      break;
#endif
    }
}

#define BC_DISPATCH(name) ((&&dispatch_##name - &&dispatch_begin))
#define BC_DISPATCH_GOTO(index) goto *(&&dispatch_begin + dispatch[index])
typedef int16_t bs_dispatch_t;

#if (INT_REG_SIZE > 32 || defined(CONFIG_MUTEK_BYTECODE_VM64))
# define BC_CLAMP32(x) do { if (ctx->sandbox) (x) = (uint32_t)(x); } while (0)
#else
# define BC_CLAMP32(x) do { } while (0)
#endif

bc_error_t bc_set_sandbox_pc(struct bc_context_s *ctx, uint32_t pc)
{
  assert(ctx->sandbox);
  const struct bc_descriptor_s * __restrict__ desc = ctx->desc;
  size_t size = desc->flags & BC_FLAGS_SIZEMASK;

  if (pc >= size || pc & 1)
    return -ERANGE;

#if CONFIG_MUTEK_BYTECODE_BREAKPOINTS > 0
  ctx->bp_skip = 0;
#endif
  ctx->vpc = (uint8_t*)ctx->desc->code + pc;
  return 0;
}

void *
bc_translate_addr(struct bc_context_s *ctx,
                  bc_reg_t addr_, size_t size,
                  bc_bool_t writable)
{
  const struct bc_descriptor_s * __restrict__ desc = ctx->desc;
  uintptr_t addr = addr_;
  uint32_t end = addr + size;

  if (ctx->sandbox)
    {
      if (end < addr)
	return NULL;

      if (addr & 0x80000000)    /* rw data segment */
	{
	  uintptr_t m = ctx->data_addr_mask;

	  addr &= m;
	  if (addr + size > m)
	    return NULL;

	  addr += ctx->data_base;
	}
      else                      /* code segment */
	{
	  if (writable)
	    return NULL;

	  if (end > (desc->flags & BC_FLAGS_SIZEMASK))
	    return NULL;

	  addr += (uintptr_t)desc->code;
	}
    }

  return (void*)addr;
}

__attribute__((noinline))
static uint_fast8_t bc_run_ldst(const struct bc_descriptor_s * __restrict__ desc,
                                struct bc_context_s *ctx, const uint16_t *pc,
                                uint16_t op)
{
 dispatch_begin:;
  bc_bool_t sandbox = ctx->sandbox;
  bc_reg_t *dst = &ctx->v[op & 0xf], d = *dst;
  op >>= 4;
  bc_reg_t *addrp = &ctx->v[op & 0xf];
  uintptr_t addr = *addrp;
  op >>= 4;
  uint_fast8_t inc = op & 1;
  uint_fast8_t w = 1 << ((op >> 2) & 3);

  if (op & 16)
    {
      if (inc)                  /* BC_LDnE/BC_STnE */
        {
          addr += (intptr_t)load16_le(pc);
        }
      else                      /* BC_STnD */
        {
          *addrp -= w;
          BC_CLAMP32(*addrp);
          addr = *addrp;
        }
    }
  else if (inc)              /* BC_LDnI/BC_STnI */
    {
      *addrp += w;
      BC_CLAMP32(*addrp);
    }

  if (sandbox)
    {
      if (addr & 0x80000000)    /* rw data segment */
        {
          addr &= ctx->data_addr_mask;
          addr += ctx->data_base;
        }
      else                      /* code segment */
        {
          if (op & 2 /* store */)
            return 1;

          size_t s = desc->flags & BC_FLAGS_SIZEMASK;
          if (addr + w > s)
            return 1;

          /* address translation */
          addr += (uintptr_t)desc->code;
        }

      if (addr & (w - 1))       /* not aligned */
        return 1;
    }

  do {
    static const bs_dispatch_t dispatch[8] = {
      BC_DISPATCH(LD8),      BC_DISPATCH(ST8),
      BC_DISPATCH(LD16),     BC_DISPATCH(ST16),
      BC_DISPATCH(LD32),     BC_DISPATCH(ST32),
      BC_DISPATCH(LD64),     BC_DISPATCH(ST64),
    };
    BC_DISPATCH_GOTO((op >> 1) & 7);

  dispatch_LD8:
    d = *(uint8_t*)addr;
    break;
  dispatch_LD16:
    d = *(uint16_t*)addr;
#if defined (CONFIG_CPU_ENDIAN_BIG)
    if (sandbox)
      d = endian_swap16(d);
#endif
    break;
  dispatch_LD32:
    d = *(uint32_t*)addr;
#if defined (CONFIG_CPU_ENDIAN_BIG)
    if (sandbox)
      d = endian_swap32(d);
#endif
    break;
  dispatch_LD64:
    if (sandbox)
      return 1;
    d = *(uint64_t*)addr;
    break;
  dispatch_ST8:
    *(uint8_t*)addr = d;
    return 0;
  dispatch_ST16:
#if defined (CONFIG_CPU_ENDIAN_BIG)
    if (sandbox)
      d = endian_swap16(d);
#endif
    *(uint16_t*)addr = d;
    return 0;
  dispatch_ST32:
#if defined (CONFIG_CPU_ENDIAN_BIG)
    if (sandbox)
      d = endian_swap32(d);
#endif
    *(uint32_t*)addr = d;
    return 0;
  dispatch_ST64:
    if (sandbox)
      return 1;
    *(uint64_t*)addr = d;
    return 0;
  } while (0);

  *dst = d;
  return 0;
}

__attribute__((noinline))
static bc_bool_t bc_run_alu(struct bc_context_s *ctx, uint16_t op)
{
  dispatch_begin:;
  bc_bool_t sandbox = ctx->sandbox;
  uint8_t a = op & 15;
  op >>= 4;
  uint8_t b = op & 15;
  bc_reg_t *dstp = &ctx->v[a];
  bc_reg_t dst = *dstp;
  bc_reg_t *srcp = &ctx->v[b];
  bc_reg_t src = *srcp;

  do {
    static const bs_dispatch_t dispatch[32] = {
      BC_DISPATCH(EQ),         BC_DISPATCH(EQ0),
      BC_DISPATCH(MOV),        BC_DISPATCH(NEQ0),
      BC_DISPATCH(LT),         BC_DISPATCH(EXTS8),
      BC_DISPATCH(LTS),        BC_DISPATCH(EXTS16),
      BC_DISPATCH(LTEQ),       BC_DISPATCH(EXTS32),
      BC_DISPATCH(LTEQS),      BC_DISPATCH(RES),
      BC_DISPATCH(ADD),        BC_DISPATCH(RES),
      BC_DISPATCH(SUB),        BC_DISPATCH(NEG),
      BC_DISPATCH(OR),         BC_DISPATCH(RAND),
      BC_DISPATCH(XOR),        BC_DISPATCH(CCALL),
      BC_DISPATCH(AND),        BC_DISPATCH(RES),
      BC_DISPATCH(SHA),        BC_DISPATCH(NOT),
      BC_DISPATCH(SHL),        BC_DISPATCH(RES),
      BC_DISPATCH(SHR),        BC_DISPATCH(RES),
      BC_DISPATCH(MUL),        BC_DISPATCH(MUL),
      BC_DISPATCH(DIV),        BC_DISPATCH(MSBS)
    };
    BC_DISPATCH_GOTO(((op >> 3) & 0x01e) | (a == b));

  dispatch_EQ:
    return (dst != src) ^ (a < b);
  dispatch_EQ0:
    return dst != 0;
  dispatch_MOV:
    dst = src;
    break;
  dispatch_NEQ0:
    return dst == 0;
  dispatch_LT:
    return dst >= src;
  dispatch_EXTS8:
    dst = (bc_sreg_t)(int8_t)src;
    break;
  dispatch_LTS:
    return (bc_sreg_t)dst >= (bc_sreg_t)src;
  dispatch_EXTS16:
    dst = (bc_sreg_t)(int16_t)src;
    break;
  dispatch_LTEQ:
    return dst > src;
  dispatch_EXTS32:
    dst = (bc_sreg_t)(int32_t)src;
    break;
  dispatch_LTEQS:
    return (bc_sreg_t)dst > (bc_sreg_t)src;
  dispatch_ADD:
    dst += src;
    break;
  dispatch_NEG:
    dst = 0;
  dispatch_SUB:
    dst -= src;
    break;
  dispatch_OR:
    dst = (uint32_t)(dst | src);
    break;
  dispatch_RAND:
    dst = (uint32_t)rand();
    break;
  dispatch_XOR:
    dst = (uint32_t)(dst ^ src);
    break;
  dispatch_CCALL:
    if (!sandbox)
      ((bc_ccall_function_t*)(uintptr_t)src)(ctx);
    break;
  dispatch_AND:
    dst = (uint32_t)(dst & src);
    break;
  dispatch_SHA:
    dst = (uint32_t)((int32_t)dst >> src);
    break;
  dispatch_NOT:
    dst = (uint32_t)~src;
    break;
  dispatch_SHL:
    dst = (uint32_t)(dst << src);
    break;
  dispatch_SHR:
    dst = (uint32_t)(dst >> src);
    break;
  dispatch_MUL:
    dst = (uint32_t)(dst * src);
    break;
  dispatch_DIV:
    if (src == 0 && sandbox)
      return 0;
    *dstp = (uint32_t)dst / (uint32_t)src;
    *srcp = (uint32_t)dst - (uint32_t)*dstp * (uint32_t)src;
    return 0;
  dispatch_MSBS:
    dst = sizeof(int) * 8 - 1 - __builtin_clz((uint32_t)dst);
    break;
  dispatch_RES:
    return 0;
  } while (0);

  BC_CLAMP32(dst);
  *dstp = dst;
  return 0;
}

bc_opcode_t bc_run_vm(struct bc_context_s *ctx)
{
  const struct bc_descriptor_s * __restrict__ desc = ctx->desc;
  const uint16_t *pc = (void*)(ctx->pc & (intptr_t)-2);
  bc_bool_t skip = ctx->pc & 1;
  uint16_t op = 0;

  int_fast16_t max_cycles = ctx->max_cycles;
  bc_bool_t sandbox = ctx->sandbox;

  if (desc->flags & BC_FLAGS_NATIVE)
    return BC_RUN_STATUS_FAULT;
  if (!!(desc->flags & BC_FLAGS_SANDBOX) ^ sandbox)
    return BC_RUN_STATUS_FAULT;

  const size_t size = desc->flags & BC_FLAGS_SIZEMASK;
  const uint16_t *code_end = (uint16_t*)desc->code + (size >> 1);
  const uintptr_t code_offset = sandbox ? (uintptr_t)desc->code : 0;

  for (;; pc++)
    {
      /* check pc upper bound */
      if (pc + 1 > code_end)
        goto err_pc;

      op = load16_le(pc);

      /* get number of extra words in the instruction */
      uint_fast8_t cst_len = 0;
      if ((op & 0xf000) == 0x7000)
        {
          uint_fast8_t s = (op >> 5) & 31;

          if ((0xff00fffe >> s) & 1)
            {
              cst_len = 1 + ((op & 0x0700) == 0x0400);

	      /* check upper bound again with extra words */
	      if (sandbox && pc + 1 + cst_len > code_end)
		goto err_pc;
            }
        }

      if (skip)
        {
          /* skip embedded constant value words if any */
          pc += cst_len;
          skip = 0;
          continue;
        }

#if CONFIG_MUTEK_BYTECODE_BREAKPOINTS > 0
      uint16_t bp_mask = ctx->bp_mask;
      uint16_t bp_skip = ctx->bp_skip;
      ctx->bp_skip = 0;
      if (bp_mask && !bp_skip)
        {
          uint_fast8_t i;
          for (i = 0; bp_mask && i < CONFIG_MUTEK_BYTECODE_BREAKPOINTS; i++)
            {
              uint16_t m = 1 << i;
              uintptr_t bp = ctx->bp_list[i];
              if (bp_mask & m)
                {
                  if ((uintptr_t)pc == bp)
                    {
                      ctx->vpc = pc;
                      ctx->bp_skip = 1;
                      return BC_RUN_STATUS_BREAK;
                    }
                  bp_mask ^= m;
                }
            }
        }
#endif

      if (sandbox)
        {
          if (max_cycles == 0)
            {
              ctx->vpc = pc;
              ctx->max_cycles = 0;
              return BC_RUN_STATUS_CYCLES;
            }
          max_cycles--;
        }

      if (ctx->trace)
        {
	  if (ctx->trace_regs)
	    bc_dump_regs(ctx);
	  bc_dump_op(ctx, pc);
	}

      /* custom op */
      if (op & 0x8000)
        {
          ctx->vpc = pc + 1;
	  if (sandbox)
	    ctx->max_cycles = max_cycles;
          return op;
        }

      bc_reg_t *dstp = &ctx->v[op & 0xf];

      do {
	static const bs_dispatch_t dispatch[8] = {
	  BC_DISPATCH(add8),
          BC_DISPATCH(cst8),
	  BC_DISPATCH(jmp),
          BC_DISPATCH(loop_pack),
	  BC_DISPATCH(alu),
          BC_DISPATCH(fmt2),
	  BC_DISPATCH(ldst),
          BC_DISPATCH(cstn_call),
	};
	BC_DISPATCH_GOTO((op >> 12) & 0x7);

      dispatch_begin:
      dispatch_add8: {
          int8_t x = (op >> 4) & 0xff;
          if (x)
            {
              *dstp += (bc_sreg_t)x;
              BC_CLAMP32(*dstp);
              break;
            }
          if (op == 0)
            {
              ctx->vpc = pc;
	      if (sandbox)
		ctx->max_cycles = max_cycles;
              return BC_RUN_STATUS_END;
            }
          else if (op == 1)
            bc_dump_pc(ctx, pc);
          else if (op & 8)
            {
              ctx->trace = op & 1;
              ctx->trace_regs = (op & 2) >> 1;
            }
          else if (op & 2)
            {
              if ((op & 1) && !sandbox)   /* abort */
                abort();
              goto err_die;   /* die */
            }
          break;
        }

      dispatch_cst8:
	*dstp = (bc_reg_t)((op >> 4) & 0xff);
	break;

      dispatch_jmp: {
          int8_t disp = op >> 4;
          if (disp)             /* jmp* */
            {
              if (op & 0xf)     /* call8 */
                *dstp = (uintptr_t)pc - code_offset;
              pc += (intptr_t)disp;
            }
          else                  /* ret */
            {
              pc = (void*)(uintptr_t)(code_offset + *dstp);
            }
          goto check_pc;
        }

      dispatch_loop_pack: {
          if (op & 0x800)         /* packing */
            {
              uint_fast8_t c = ((op >> 8) & 0x7) + 1;
              uint_fast8_t r = (op & 0xf);
	      if (sandbox && r + c > 16)
		break;
              assert(r + c <= 16);
              bc_run_packing(ctx, c, r, (op >> 4) & 0xf);
              break;
            }

          /* loop */
	  int8_t d = op >> 3;
          d >>= 1;
	  if (d < 0)
	    {
	      if (!--(*dstp))
		break;
              BC_CLAMP32(*dstp);
	    }
	  else if (*dstp > 0)
	    {
	      (*dstp)--;
	      break;
	    }
	  pc += d;
          goto check_pc;
	}

      dispatch_alu:
	skip = bc_run_alu(ctx, op);
	break;

      dispatch_fmt2: {
	  uint_fast8_t bit = (op >> 4) & 0x1f;
          bc_reg_t dst = *dstp;
	  if (op & 0x0800)
            {
              if (op & 0x0400)
                {
                  if (op & 0x0200) /* BC_EXTZ */
                    dst &= 0xffffffff >> bit;
                  else
                    dst = (uint32_t)((int32_t)dst >> bit); /* BC_SHIA */
                }
              else
                {
                  dst = op & 0x0200 ? (uint32_t)(dst >> bit)
                                   : (uint32_t)(dst << bit); /* BC_SHI* */
                }
            }
          else
            {
              bc_reg_t mask = 1U << bit;
              bc_reg_t vmask = op & 0x0200 ? mask : 0;
              if (op & 0x0400)
                dst = (dst & ~mask) | vmask; /* BC_BIT* */
              else
                skip = ((dst ^ vmask) >> bit) & 1; /* BC_TST* */
            }
          BC_CLAMP32(dst);
          *dstp = dst;
	  break;
	}

      dispatch_cstn_call: {
	  if ((op & 0x0300) == 0x0000) /* not ld/st */
	    {
              if ((op & 0x00e0) == 0x0000)
                {
                  if (op & 0x0010) /* mode */
                    {
                      ctx->mode = ((op & 0x0c00) >> 6) | (op & 15);
                    }
                  else /* gaddr */
                    {
		      goto err_ret;
                    }
                  break;
                }

              uintptr_t rpc = (void*)pc - desc->code;

              /* fetch constant */
              bc_reg_t x = load16_le(++pc);
              if (op & 0x0400)
                x |= (uint32_t)load16_le(++pc) << 16;

              if (op & 0x0080)  /* cst16, cst32 */
                {
                  if (op & 0x0800)  /* set high */
                    {
                      if (op & 0x0400)
                        x |= 0xffffffff00000000ULL;
                      else
                        x |= 0xffffffffffff0000ULL;
                    }

                  *dstp = x << ((op & 0x0070) >> 1); /* byte shift */
                  break;
                }

              if (op & 0x0800)  /* pc relative */
                {
                  if (op & 0x0400)  /* sign extend */
                    x = (bc_sreg_t)(int32_t)x;
                  else
                    x = (bc_sreg_t)(int16_t)x;
                  x += rpc;
                }

              if (op & 0x0020)  /* laddr */
                {
                    if (sandbox)
                      x |= (op & 0x40) << 25; /* bit 31 */
                    else
		      x += (uintptr_t)desc->code;
                  *dstp = x;
                  break;
                }

              /* call/jmp */
              if (op & 0x0010)     /* save return address */
                *dstp = (uintptr_t)pc - code_offset + 2;

              pc = (const uint16_t*)(desc->code + x);
              goto check_pc;
	    }
        }

      dispatch_ldst:
	if (bc_run_ldst(desc, ctx, pc + 1, op))
          goto err_ret;
	break;

      check_pc:
        if (pc + 1 < (uint16_t*)desc->code || ((uintptr_t)pc & 1))
          goto err_pc;
	break;

      } while (0);

    }

 err_pc:
  if (sandbox)
    goto err_ret;
  assert(!"bytecode pc out of range");

 err_die:
  fprintf(stderr, "bytecode: die %p\n", pc);
  bc_dump_pc(ctx, pc);

 err_ret:
  ctx->vpc = pc;
  if (sandbox)
    ctx->max_cycles = max_cycles;
  return BC_RUN_STATUS_FAULT;
}
