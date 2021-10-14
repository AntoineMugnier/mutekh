/* -*- mode: c -*-
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

#undef LOGK_MODULE_ID
#define LOGK_MODULE_ID "bcvm"

#include <mutek/printk.h>
#include <mutek/bytecode.h>

#include <hexo/endian.h>
#include <hexo/bit.h>
#include <stdlib.h>

#include <stdarg.h>

void
bc_set_regs_va(struct bc_context_s *ctx, uint16_t mask, va_list ap)
{
  while (mask)
    {
      uint_fast8_t r = bit_ctz(mask);
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

bc_opcode_t bc_run_vm(struct bc_context_s *ctx);
bc_opcode_t bc_run_sandbox(struct bc_context_s *ctx);

error_t
bc_desc_init(struct bc_descriptor_s *desc,
             const void *code, size_t len,
             enum bc_flags_s flags)
{
  assert(!((uintptr_t)code & 1));
  if (flags & BC_FLAGS_SANDBOX)
#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
    desc->run = &bc_run_sandbox;
#else
    return -ENOTSUP;
#endif
  else
    desc->run = &bc_run_vm;
  desc->code = code;
  desc->flags = flags | len;
  return 0;
}

error_t
bc_load(struct bc_descriptor_s *desc,
        const uint8_t *blob, size_t len)
{
  if ((uintptr_t)blob & 1)
    return -ENOTSUP;

  if (len < /* header */ 4)
    return -EINVAL;

  uint32_t flags = endian_le32_na_load(blob);
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
#ifdef CONFIG_MUTEK_BYTECODE_NATIVE
  ctx->skip = 0;
#endif
  ctx->desc = desc;
#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
  ctx->sandbox = 0;
  ctx->max_cycles = -1;
#endif
#ifdef CONFIG_MUTEK_BYTECODE_TRACE
  ctx->trace = BC_TRACE_DISABLED;
#endif
#if CONFIG_MUTEK_BYTECODE_BREAKPOINTS > 0
  ctx->bp_mask = 0;
  ctx->bp_skip = 0;
#endif
}

#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
void
bc_init_sandbox(struct bc_context_s *ctx, const struct bc_descriptor_s *desc,
                void *data_base, size_t data_size,
                uint_fast16_t max_cycles)
{
  ctx->vpc = desc->code;
  ctx->mode = 0;
#ifdef CONFIG_MUTEK_BYTECODE_NATIVE
  ctx->skip = 0;
#endif
  ctx->desc = desc;
  ctx->sandbox = 1;
  ctx->max_cycles = max_cycles;
  ctx->data_base = data_base;
  ctx->data_end = data_size + 0x80000000;
#ifdef CONFIG_MUTEK_BYTECODE_TRACE
  ctx->trace = BC_TRACE_DISABLED;
#endif
#if CONFIG_MUTEK_BYTECODE_BREAKPOINTS > 0
  ctx->bp_mask = 0;
  ctx->bp_skip = 0;
#endif
}
#endif

#if defined(CONFIG_MUTEK_BYTECODE_VM)
const char * bc_opname(uint16_t op)
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
    { 0xfff8, 0x0008, "trace" },
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
    { 0xfff0, 0x7010, "mode" },
    { 0xfff0, 0x7000, "gaddr" },
    { 0xfff0, 0x7400, "pick" },
    { 0xfff0, 0x7410, "place" },
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
#endif

#ifdef CONFIG_MUTEK_BYTECODE_DEBUG
static void bc_dump_op(const char *cap,
                       const struct bc_context_s *ctx,
                       const uint16_t *pc)
{
  __unused__ uint8_t mode = ctx->mode;
# ifdef CONFIG_MUTEK_BYTECODE_NATIVE
  if (ctx->desc->flags & BC_FLAGS_NATIVE)
    {
      logk_debug("%s: pc=%p, mode=%u", cap, pc, mode);
    }
  else
# endif
    {
# ifdef CONFIG_MUTEK_BYTECODE_VM
      const void *code = ctx->desc->code;
      size_t size = ctx->desc->flags & BC_FLAGS_SIZEMASK;

      if (pc >= (uint16_t*)code &&
          (uint8_t*)pc < (uint8_t*)code + size &&
          !((uintptr_t)pc & 1))
        {
          uint16_t op = endian_le16(*pc);

          logk_debug("%s: pc=%p (%u), opcode=%04x (%s), mode=%u",
                     cap, pc, pc - (uint16_t*)code,
                     op, bc_opname(op), mode);
        }
      else
        {
          logk_debug("%s: pc=%p (%u), mode=%u",
                     cap, pc, pc - (uint16_t*)code, mode);
        }
# endif
    }
}

static void bc_dump_regs(const struct bc_context_s *ctx)
{
  uint_fast8_t i;
  for (i = 0; i < 16; i += 4)
    logk_debug("r%02u: %" BC_REG_FORMAT" %" BC_REG_FORMAT" %" BC_REG_FORMAT" %" BC_REG_FORMAT,
               i, ctx->v[i], ctx->v[i + 1], ctx->v[i + 2], ctx->v[i + 3]);
}

static void bc_dump_pc(const struct bc_context_s *ctx, const uint16_t *pc)
{
  bc_dump_op("dump", ctx, pc);
  bc_dump_regs(ctx);
}

#endif

#ifdef CONFIG_MUTEK_BYTECODE_TRACE
static void bc_trace_pc(const char *cap,
                        const struct bc_context_s *ctx,
                        const uint16_t *pc)
{
  bc_dump_op(cap, ctx, pc);
  if (ctx->trace & _BC_TRACE_REGS)
    bc_dump_regs(ctx);
}
#endif

void bc_dump(const struct bc_context_s *ctx, bool_t regs)
{
#ifdef CONFIG_MUTEK_BYTECODE_DEBUG
  bc_dump_op("dump", ctx, (void*)(ctx->pc & (intptr_t)-2));
  if (regs)
    bc_dump_regs(ctx);
#endif
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

#ifdef CONFIG_MUTEK_BYTECODE_NATIVE

extern inline void
bc_unpack_op8(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_unpack8(t, c);
}

extern inline void
bc_unpack_swap_op16(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_unpack16(t, c);
  bc_swap16(t, c);
}

extern inline void
bc_unpack_op16(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_unpack16(t, c);
}

extern inline void
bc_pack_op8(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_pack8(t, c);
}

extern inline void
bc_swap_pack_op16(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_swap16(t, c);
  bc_pack16(t, c);
}

extern inline void
bc_pack_op16(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_pack16(t, c);
}

extern inline void
bc_swap_pack_op32(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_swap32(t, c);
# if INT_REG_SIZE > 32 || defined(CONFIG_MUTEK_BYTECODE_VM64)
  bc_pack32(t, c);
# endif
}

# if INT_REG_SIZE > 32 || defined(CONFIG_MUTEK_BYTECODE_VM64)
extern inline void
bc_pack_op32(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_pack32(t, c);
}
# endif

extern inline void
bc_unpack_swap_op32(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
# if INT_REG_SIZE > 32 || defined(CONFIG_MUTEK_BYTECODE_VM64)
  bc_unpack32(t, c);
# endif
  bc_swap32(t, c);
}

# if INT_REG_SIZE > 32 || defined(CONFIG_MUTEK_BYTECODE_VM64)
extern inline void
bc_unpack_op32(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_unpack32(t, c);
}
# endif

extern inline void
bc_swap_op16(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_swap16(t, c);
}

extern inline void
bc_swap_op32(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_swap32(t, c);
}

#endif

/* Select the bytes that have a corresponding bit set in the mask and
   move them to the begining of the array. Other bytes are moved to
   the end of the array. */
extern inline
void bc_pick(void *b_, uint16_t x)
{
  uint8_t *b = b_;
  uint8_t out[16];
  uint_fast8_t i = 0, j = 0, k = bit_popc16(x);
  while (x)
    {
      out[x & 1 ? j++ : k++] = b[i++];
      x >>= 1;
    }
  memcpy(b, out, k);
}

/* Move contiguous bytes that are in the packed byte array to the
   locations corresponding to the bits set in the mask. Bytes
   corresponding to the cleared bits in the mask are set to 0. */
extern inline
void bc_place(void *b_, uint16_t x)
{
  uint8_t *b = b_;
  uint_fast8_t l = bit_clz16(x);
  uint_fast8_t j = 16 - l;
  uint_fast8_t i = bit_popc16(x);

  for (x <<= l; j--; x <<= 1)
    {
      if (x & 0x8000)
	b[j] = b[--i];
      else
	b[j] = 0;
    }
}

#ifdef CONFIG_MUTEK_BYTECODE_VM

#define BC_DISPATCH(name) ((&&dispatch_##name - &&dispatch_begin))
#define BC_DISPATCH_GOTO(index) goto *(&&dispatch_begin + dispatch[index])
typedef int16_t bs_dispatch_t;

#if (INT_REG_SIZE > 32 || defined(CONFIG_MUTEK_BYTECODE_VM64)) \
  && defined(CONFIG_MUTEK_BYTECODE_SANDBOX)
# define BC_CLAMP32(sandbox, x) do { if (sandbox) (x) = (uint32_t)(x); } while (0)
#else
# define BC_CLAMP32(sandbox, x) do { } while (0)
#endif

#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX

error_t bc_set_sandbox_pc(struct bc_context_s *ctx, uint32_t pc)
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

inline void *
bc_translate_addr(struct bc_context_s *ctx,
                  bc_reg_t addr_, size_t size,
                  bool_t writable)
{
  const struct bc_descriptor_s * __restrict__ desc = ctx->desc;
  uintptr_t addr = addr_;
  uint32_t end = addr + size;

  if (end < addr)
    return NULL;

  if (addr & 0x80000000)    /* rw data segment */
    {
      if (end > ctx->data_end)
        return NULL;

      addr = addr - 0x80000000 + (uintptr_t)ctx->data_base;
    }
  else                      /* code segment */
    {
      if (writable)
        return NULL;

      if (end > (desc->flags & BC_FLAGS_SIZEMASK))
        return NULL;

      addr += (uintptr_t)desc->code;
    }

  return (void*)addr;
}

#endif

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

#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
# define BC_CONFIG_SANDBOX(...) __VA_ARGS__
# define BC_CONFIG_NOT_SANDBOX(...)
#else
# define BC_CONFIG_SANDBOX(...)
# define BC_CONFIG_NOT_SANDBOX(...) __VA_ARGS__
#endif

#if CONFIG_MUTEK_BYTECODE_BREAKPOINTS > 0
# define BC_CONFIG_BREAKPOINTS(...) __VA_ARGS__
#else
# define BC_CONFIG_BREAKPOINTS(...)
#endif

#ifdef CONFIG_MUTEK_BYTECODE_TRACE
# define BC_CONFIG_TRACE(...) __VA_ARGS__
#else
# define BC_CONFIG_TRACE(...)
#endif

#ifdef CONFIG_MUTEK_BYTECODE_NATIVE
# define BC_CONFIG_NATIVE(...) __VA_ARGS__
#else
# define BC_CONFIG_NATIVE(...)
#endif

#ifdef CONFIG_MUTEK_BYTECODE_DEBUG
# define BC_CONFIG_DEBUG(...) __VA_ARGS__
#else
# define BC_CONFIG_DEBUG(...)
#endif

#if INT_REG_SIZE > 32 || defined(CONFIG_MUTEK_BYTECODE_VM64)
# define BC_CONFIG_64(...) __VA_ARGS__
# define BC_CONFIG_NOT_64(...)
#else
# define BC_CONFIG_64(...)
# define BC_CONFIG_NOT_64(...) __VA_ARGS__
#endif

#if INT_PTR_SIZE == 16
# define BC_NATIVE_PTR_NA_LOAD(x) endian_16_na_load(x)
#endif
#if INT_PTR_SIZE == 32
# define BC_NATIVE_PTR_NA_LOAD(x) endian_32_na_load(x)
#endif
#if INT_PTR_SIZE == 64
# define BC_NATIVE_PTR_NA_LOAD(x) endian_64_na_load(x)
#endif

inline bool_t
bc_test_bp(const struct bc_context_s *ctx, const void *pc)
{
#if CONFIG_MUTEK_BYTECODE_BREAKPOINTS > 0
  uint16_t bp_mask = ctx->bp_mask;
  uint_fast8_t i;
  for (i = 0; bp_mask && i < CONFIG_MUTEK_BYTECODE_BREAKPOINTS; i++)
    {
      uint16_t m = 1 << i;
      uintptr_t bp = ctx->bp_list[i];

      if (bp_mask & m)
        {
          if ((uintptr_t)pc == bp)
            return 1;
          bp_mask ^= m;
        }
    }
#endif
  return 0;
}

/* backslash-region-begin */
#define BC_VM_GEN(fcname, sandbox)
__attribute__((noinline))
static bool_t bc_run_##fcname##_ldst(const struct bc_descriptor_s * __restrict__ desc,
                                     struct bc_context_s *ctx, const uint16_t *pc,
                                     uint16_t op)
{
  dispatch_begin:;
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
          addr += (intptr_t)(int16_t)endian_le16(*pc);
        }
      else                      /* BC_STnD */
        {
          *addrp -= w;
          BC_CLAMP32(sandbox, *addrp);
          addr = *addrp;
        }
    }
  else if (inc)              /* BC_LDnI/BC_STnI */
    {
      *addrp += w;
      BC_CLAMP32(sandbox, *addrp);
    }

  BC_CONFIG_SANDBOX(
    if (sandbox)
      {
        void *a = bc_translate_addr(ctx, addr, w, op & 2);
        if (!a)
          return 1;
        addr = (uintptr_t)a;

        if (addr & (w - 1))       /* not aligned */
          return 1;
      }
  );

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
  BC_CONFIG_SANDBOX(
    if (sandbox)
      d = endian_le16(d);
  );
    break;
  dispatch_LD32:
    d = *(uint32_t*)addr;
  BC_CONFIG_SANDBOX(
    if (sandbox)
      d = endian_le32(d);
  );
    break;
  dispatch_LD64:
  BC_CONFIG_SANDBOX(
    if (sandbox)
      return 1;
  );
    d = *(uint64_t*)addr;
    break;
  dispatch_ST8:
    *(uint8_t*)addr = d;
    return 0;
  dispatch_ST16:
  BC_CONFIG_SANDBOX(
    if (sandbox)
      d = endian_le16(d);
  );
    *(uint16_t*)addr = d;
    return 0;
  dispatch_ST32:
  BC_CONFIG_SANDBOX(
    if (sandbox)
      d = endian_le32(d);
  );
    *(uint32_t*)addr = d;
    return 0;
  dispatch_ST64:
  BC_CONFIG_SANDBOX(
    if (sandbox)
      return 1;
  );
    *(uint64_t*)addr = d;
    return 0;
  } while (0);

  *dst = d;
  return 0;
}

__attribute__((noinline))
static bool_t bc_run_##fcname##_alu(struct bc_context_s *ctx, uint16_t op)
{
  dispatch_begin:;
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
    BC_CONFIG_64( dst = (bc_sreg_t)(int32_t)src );
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
    dst = (uint32_t)rand_64();
    break;
  dispatch_XOR:
    dst = (uint32_t)(dst ^ src);
    break;
  dispatch_CCALL:
  BC_CONFIG_SANDBOX(
      if (!sandbox)
  )
       dst = ((bc_ccall_function_t*)(uintptr_t)src)(ctx);
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
    BC_CONFIG_SANDBOX(
      if (src == 0 && sandbox)
        return 0;
    );
    *dstp = (uint32_t)dst / (uint32_t)src;
    *srcp = (uint32_t)dst - (uint32_t)*dstp * (uint32_t)src;
    return 0;
  dispatch_MSBS:
    dst = bit_msb_index((uint32_t)dst);
    break;
  dispatch_RES:
    return 0;
  } while (0);

  BC_CLAMP32(sandbox, dst);
  *dstp = dst;
  return 0;
}

bc_opcode_t bc_run_##fcname(struct bc_context_s *ctx)
{
  const struct bc_descriptor_s * __restrict__ desc = ctx->desc;
  const uint16_t *pc = (void*)(ctx->pc & (intptr_t)-2);
  bool_t skip = ctx->pc & 1;
  bc_opcode_t op = 0;
  bc_opcode_t status;

  BC_CONFIG_SANDBOX(
    int_fast16_t max_cycles = ctx->max_cycles;

    if (desc->flags & BC_FLAGS_NATIVE)
      return BC_RUN_STATUS_FAULT;
    if (!!(desc->flags & BC_FLAGS_SANDBOX) ^ sandbox)
      return BC_RUN_STATUS_FAULT;

    const size_t size = desc->flags & BC_FLAGS_SIZEMASK;
    const uint16_t *code_end = (uint16_t*)desc->code + (size >> 1);
    const uintptr_t code_offset = sandbox ? (uintptr_t)desc->code : 0;
  );

  BC_CONFIG_NOT_SANDBOX(
    if (desc->flags & (BC_FLAGS_SANDBOX | BC_FLAGS_NATIVE))
      return BC_RUN_STATUS_FAULT;
    const uintptr_t code_offset = 0;
  );

  BC_CONFIG_TRACE(
    if (ctx->trace & _BC_TRACE_JUMP)
      bc_trace_pc("resume at   ", ctx, pc);
  );

  for (;; pc++)
    {
      BC_CONFIG_SANDBOX(
        /* check pc upper bound */
        if (sandbox && pc + 1 > code_end)
          goto err_pc;
      );

      op = endian_le16(*pc);

      /* get number of extra words in the instruction */
      uint_fast8_t cst_len = 0;
      if ((op & 0xf000) == 0x7000)
        {
          static const uint8_t cl[32] = {
            0x54, 0x55, 0x55, 0x55, 0x00, 0x00, 0x55, 0x55,
            0xa9, 0xaa, 0x55, 0x55, 0x00, 0x00, 0x55, 0x55,
            0x54, 0x55, 0x55, 0x55, 0x00, 0x00, 0x55, 0x55,
            0xa8, 0xaa, 0x55, 0x55, 0x00, 0x00, 0x55, 0x55
          };

          /* op: 0111 bbbb baa- ---- */
          cst_len = (cl[/* b */(op >> 7) & 0b11111]
                    >> /* a */((op >> 4) & 0b110)) & 3;

          BC_CONFIG_SANDBOX(
            /* check upper bound again with extra words */
            if (sandbox && pc + 1 + cst_len > code_end)
              goto err_pc;
          );
        }

      if (skip)
        {
          /* skip embedded constant value words if any */
          pc += cst_len;
          skip = 0;
          continue;
        }

  BC_CONFIG_BREAKPOINTS(
      uint16_t bp_skip = ctx->bp_skip;
      ctx->bp_skip = 0;
      if (ctx->bp_mask && !bp_skip &&
        bc_test_bp(ctx, pc))
        {
          ctx->bp_skip = 1;
          status = BC_RUN_STATUS_BREAK;
          goto ret;
        }
  );

  BC_CONFIG_SANDBOX(
      if (sandbox)
        {
          if (max_cycles == 0)
            {
              status = BC_RUN_STATUS_CYCLES;
              goto ret;
            }
          max_cycles--;
        }
  );

  BC_CONFIG_TRACE(
      if (ctx->trace & _BC_TRACE_ALL)
        bc_trace_pc("before", ctx, pc);
  );

      /* custom op */
      if (op & 0x8000)
        {
          BC_CONFIG_NATIVE(
            ctx->skip = 1;
          );

          pc++;
          status = op;
          goto ret;
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
          BC_DISPATCH(fmt3),
	};
	BC_DISPATCH_GOTO((op >> 12) & 0x7);

      dispatch_begin:
      dispatch_add8: {
          int8_t x = (op >> 4) & 0xff;
          if (x)                /* add8 */
            {
              *dstp += (bc_sreg_t)x;
              BC_CLAMP32(sandbox, *dstp);
              break;
            }
          if (op == 0)          /* end */
            {
              status = BC_RUN_STATUS_END;
              goto ret;
            }
        BC_CONFIG_DEBUG(
          else if (op == 1)     /* dump */
            bc_dump_pc(ctx, pc);
        )
        BC_CONFIG_TRACE(
          else if (op & 8)      /* trace */
            {
              ctx->trace = op & 7;
            }
        )
          else if (op & 2)
            {
              if ((op & 1)      /* abort */
                BC_CONFIG_SANDBOX( && !sandbox )
              )
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
          bc_reg_t link = (uintptr_t)pc - code_offset + 2;
          const uint16_t *tgt = (void*)(uintptr_t)(code_offset + *dstp - 2);
          switch ((uint8_t)disp)
            {
            case 0xff:          /* call */
              *dstp = link;
            case 0x00:          /* ret/jmp */
              pc = tgt;
              break;
            default:            /* jmp8 */
              if (op & 0xf)     /* call8 */
                *dstp = link;
              pc += (intptr_t)disp;
            }
          goto check_pc;
        }

      dispatch_loop_pack: {
          if (op & 0x800)         /* packing */
            {
              uint_fast8_t c = ((op >> 8) & 0x7) + 1;
              uint_fast8_t r = (op & 0xf);
              BC_CONFIG_SANDBOX(
                if (sandbox && r + c > 16)
                  goto err_ret;
              );
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
              BC_CLAMP32(sandbox, *dstp);
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
	skip = bc_run_##fcname##_alu(ctx, op);
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
          BC_CLAMP32(sandbox, dst);
          *dstp = dst;
	  break;
	}

      dispatch_fmt3:
	  if (op & 0x0300) /* ld/st */
            goto dispatch_ldst;

          switch (op & 0x0ce0)
            {
              case 0x0000:
                if (op & 0x0010)    /* mode */
                  {
                    ctx->mode = op & 15;
                  }
                else    /* gaddr */
                  {
                    BC_CONFIG_SANDBOX(
                      if (sandbox)
                        goto err_ret;
                      );
                    *dstp = BC_NATIVE_PTR_NA_LOAD(++pc);
                    pc += INT_PTR_SIZE / 16 - 1;
                  }
                break;

              case 0x0400: {    /* pick/place */
                uint16_t mask = endian_le16(*++pc);
                uint_fast8_t r = (op & 0xf);
                BC_CONFIG_SANDBOX(
                  if (sandbox)
                    if (!mask || r * 4 + bit_msb_index(mask) > 63)
                      goto err_ret;
                );
                if (op & 0x0010)
                  bc_place(ctx->v + r, mask);
                else
                  bc_pick(ctx->v + r, mask);
                break;
              }

              case 0x0800:    /* unused */
              case 0x0c00:
                goto err_ret;

              default: {      /* cst/laddr/jmp/call */

              uintptr_t rpc = (void*)pc - desc->code;

              /* fetch constant */
              bc_reg_t x = endian_le16(*++pc);
              if (op & 0x0400)
                x |= (uint32_t)endian_le16(*++pc) << 16;

              if (op & 0x0080)  /* cst16, cst32 */
                {
                  if (op & 0x0800)  /* set high */
                    {
                      if (op & 0x0400)
                        BC_CONFIG_64( x |= 0xffffffff00000000ULL );
                      else
                        x |= 0xffffffffffff0000ULL;
                    }

                  *dstp = x << ((op & 0x0070) >> 1); /* byte shift */
                  break;
                }

              if (op & 0x0800)  /* pc relative */
                {
                  if (op & 0x0400)  /* sign extend */
                    BC_CONFIG_64( x = (bc_sreg_t)(int32_t)x );
                  else
                    x = (bc_sreg_t)(int16_t)x;
                  x += rpc;
                }

              if (op & 0x0020)  /* laddr */
                {
                  BC_CONFIG_SANDBOX(
                    if (sandbox)
                      x |= (op & 0x40) << 25; /* bit 31 */
                    else
                 )
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
          break;

      dispatch_ldst:
	if (bc_run_##fcname##_ldst(desc, ctx, pc + 1, op))
          goto err_ret;
        pc += cst_len;
	break;

      check_pc:
        BC_CONFIG_SANDBOX(
          if (sandbox && (pc + 1 < (uint16_t*)desc->code || ((uintptr_t)pc & 1)))
            goto err_pc;
        );
        BC_CONFIG_TRACE(
          if (ctx->trace & _BC_TRACE_JUMP)
            bc_trace_pc("jump to     ", ctx, pc + 1);
         );
	break;

      } while (0);

    }

 err_pc:
  BC_CONFIG_SANDBOX(
    if (sandbox)
      goto err_ret;
  );
  assert(!"bytecode pc out of range");

 err_die:
  logk_error("die at %p", pc);

 err_ret:
  status = BC_RUN_STATUS_FAULT;
 ret:
  BC_CONFIG_TRACE(
    if (ctx->trace & _BC_TRACE_JUMP)
      bc_trace_pc("leave before", ctx, pc);
  );
  ctx->vpc = pc;
  BC_CONFIG_SANDBOX(
    if (sandbox)
      ctx->max_cycles = max_cycles;
  );
  return status;
}
/* backslash-region-end */

#ifdef CONFIG_MUTEK_BYTECODE_VM_SINGLE
# ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
BC_VM_GEN(vm, ctx->sandbox);
bc_opcode_t bc_run_sandbox(struct bc_context_s *ctx)
{
  return bc_run_vm(ctx);
}
#else
BC_VM_GEN(vm, 0);
#endif

#else /* !CONFIG_MUTEK_BYTECODE_VM_SINGLE */

# ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
BC_VM_GEN(sandbox, 1);
# endif
BC_VM_GEN(vm, 0);
#endif

#endif

