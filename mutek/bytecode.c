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

error_t
bc_load(struct bc_descriptor_s *desc,
        const uint8_t *blob, size_t len)
{
  if ((uintptr_t)blob & 1)
    return -ENOTSUP;

  if (len < /* header */ 4)
    return -EINVAL;

  size_t count = endian_le16_na_load(blob + 2);

  if (len < /* header */ 4 + count * 2)
    return -EINVAL;

  desc->code = blob + 4;
  desc->run = &bc_run_vm;
  desc->flags = endian_le16_na_load(blob);
  desc->op_count = count;

  return 0;
}

void
bc_init(struct bc_context_s *ctx,
        const struct bc_descriptor_s *desc)
{
  ctx->vpc = desc->code;
#ifdef CONFIG_MUTEK_BYTECODE_NATIVE
  ctx->skip = 0;
#endif
  ctx->desc = desc;
#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
  ctx->sandbox = 0;
  ctx->max_cycles = -1;
#endif
#ifdef CONFIG_MUTEK_BYTECODE_TRACE
  ctx->trace = 0;
  ctx->trace_regs = 0;
#endif
}

#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
void
bc_init_sandbox(struct bc_context_s *ctx, const struct bc_descriptor_s *desc,
                void *data_base, uint_fast8_t data_addr_bits,
                uint_fast32_t max_cycles)
{
  ctx->vpc = desc->code;
#ifdef CONFIG_MUTEK_BYTECODE_NATIVE
  ctx->skip = 0;
#endif
  ctx->desc = desc;
  ctx->sandbox = 1;
  ctx->max_cycles = max_cycles;
  if (data_addr_bits)
    {
      assert(data_addr_bits >= 3);
      assert(((uintptr_t)data_base & 7) == 0);
      ctx->data_base = (uintptr_t)data_base;
      ctx->data_addr_mask = bit(data_addr_bits) - 1;
    }
  else
    {
      static uint64_t dummy;
      ctx->data_base = (uintptr_t)&dummy;
      ctx->data_addr_mask = 7;
    }
#ifdef CONFIG_MUTEK_BYTECODE_TRACE
  ctx->trace = 0;
  ctx->trace_regs = 0;
#endif
}
#endif

#if defined(CONFIG_MUTEK_BYTECODE_VM) && defined(CONFIG_MUTEK_BYTECODE_DEBUG)
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
    { 0xffff, 0x0040, "nop" },
    { 0xfffc, 0x0008, "trace" },
    { 0xf000, BC_OP_ADD8  << 8, "add8" },
    { 0xf000, BC_OP_CST8  << 8, "cst8" },
    { 0xfff0, BC_OP_JMP   << 8, "ret" },
    { 0xf00f, BC_OP_JMP   << 8, "jmp8" },
    { 0xf000, BC_OP_JMP   << 8, "call8" },
    { 0xf800, BC_OP_LOOP  << 8, "loop" },
    { 0xf800, BC_OP_PACK  << 8, "(un)pack/swap" },
    { 0xff00, BC_OP_EQ    << 8, "eq", "eq0" },
    { 0xff00, BC_OP_NEQ   << 8, "neq", "neq0" },
    { 0xff00, BC_OP_LT    << 8, "lt" },
    { 0xff00, BC_OP_LTS   << 8, "lts" },
    { 0xff00, BC_OP_LTEQ  << 8, "lteq" },
    { 0xff00, BC_OP_LTEQS << 8, "lteqs" },
    { 0xff00, BC_OP_ADD   << 8, "add" },
    { 0xff00, BC_OP_SUB   << 8, "sub", "neg" },
    { 0xff00, BC_OP_OR    << 8, "or", "rand" },
    { 0xff00, BC_OP_XOR   << 8, "xor", "ccall" },
    { 0xff00, BC_OP_AND   << 8, "and" },
    { 0xff00, BC_OP_ANDN  << 8, "andn", "not" },
    { 0xff00, BC_OP_SHL   << 8, "shl" },
    { 0xff00, BC_OP_SHR   << 8, "shr" },
    { 0xff00, BC_OP_MUL   << 8, "mul" },
    { 0xff00, BC_OP_MOV   << 8, "mov", "msbs" },
    { 0xfe00, BC_OP_TSTC  << 8, "tstc" },
    { 0xfe00, BC_OP_TSTS  << 8, "tsts" },
    { 0xfe00, BC_OP_BITC  << 8, "bitc" },
    { 0xfe00, BC_OP_BITS  << 8, "bits" },
    { 0xfe00, BC_OP_SHIL  << 8, "shil" },
    { 0xfe00, BC_OP_SHIR  << 8, "shir" },
    { 0xfe00, BC_OP_EXTS  << 8, "exts" },
    { 0xfe00, BC_OP_EXTZ  << 8, "extz" },
    { 0xf900, BC_OP_LD    << 8, "ld" },
    { 0xf900, BC_OP_LDI   << 8, "ldi" },
    { 0xf900, BC_OP_ST    << 8, "st" },
    { 0xf900, BC_OP_STI   << 8, "sti" },
    { 0xfff0, BC_OP_CST  << 8, "gaddr" },
    { 0xf9f0, BC_OP_CST  << 8, "laddr" },
    { 0xffff, (BC_OP_CALL  << 8) | 0x10, "jmp32" },
    { 0xfff0, (BC_OP_CALL  << 8) | 0x10, "call32" },
    { 0xf900, BC_OP_CST   << 8, "cst" },
    { 0xf900, BC_OP_STD   << 8, "std" },
    { 0xf900, BC_OP_LDE   << 8, "lde" },
    { 0xf900, BC_OP_STE   << 8, "ste" },
    { 0x0000, 0x0000, "invalid" },
  };
  uint_fast8_t i;
  for (i = 0; ; i++)
    if ((op & ops[i].mask) == ops[i].op)
      return !ops[i].name1 || ((op ^ (op >> 4)) & 15) ? ops[i].name : ops[i].name1;
  return NULL;
}
#endif

static void bc_dump_(const struct bc_context_s *ctx, const uint16_t *pc, bool_t regs)
{
#ifdef CONFIG_MUTEK_BYTECODE_DEBUG
  uint_fast8_t i;

# ifdef CONFIG_MUTEK_BYTECODE_NATIVE
  if (ctx->desc->flags & BC_FLAGS_NATIVE)
    {
      printk("bytecode: pc=%p", pc);
    }
  else
# endif
    {
# ifdef CONFIG_MUTEK_BYTECODE_VM

      printk("bytecode: pc=%p (%u)", pc, pc - (uint16_t*)ctx->desc->code);

      if (pc >= (uint16_t*)ctx->desc->code &&
          pc < (uint16_t*)ctx->desc->code + ctx->desc->op_count &&
          !((uintptr_t)pc & 1))
        {
          uint16_t op = endian_le16(*pc);

          printk(", opcode=%04x (%s)", op, bc_opname(op));
        }
# endif
    }

  printk("\n");

  if (regs)
    for (i = 0; i < 16; i++)
      printk("r%02u=%" BC_REG_FORMAT "%c", i, ctx->v[i], (i + 1) % 4 ? ' ' : '\n');
#endif
}

void bc_dump(const struct bc_context_s *ctx, bool_t regs)
{
  bc_dump_(ctx, (void*)(ctx->pc & (intptr_t)-2), regs);
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

void bc_unpack_op8(struct bc_context_s *ctx, reg_t op);
void bc_unpack_op8(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_unpack8(t, c);
}

void bc_unpack_swap_op16(struct bc_context_s *ctx, reg_t op);
void bc_unpack_swap_op16(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_unpack16(t, c);
  bc_swap16(t, c);
}

void bc_unpack_op16(struct bc_context_s *ctx, reg_t op);
void bc_unpack_op16(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_unpack16(t, c);
}

void bc_pack_op8(struct bc_context_s *ctx, reg_t op);
void bc_pack_op8(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_pack8(t, c);
}

void bc_swap_pack_op16(struct bc_context_s *ctx, reg_t op);
void bc_swap_pack_op16(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_swap16(t, c);
  bc_pack16(t, c);
}

void bc_pack_op16(struct bc_context_s *ctx, reg_t op);
void bc_pack_op16(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_pack16(t, c);
}

void bc_swap_pack_op32(struct bc_context_s *ctx, reg_t op);
void bc_swap_pack_op32(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_swap32(t, c);
# if INT_REG_SIZE > 32 || defined(CONFIG_MUTEK_BYTECODE_VM64)
  bc_pack32(t, c);
# endif
}

# if INT_REG_SIZE > 32 || defined(CONFIG_MUTEK_BYTECODE_VM64)
void bc_pack_op32(struct bc_context_s *ctx, reg_t op);
void bc_pack_op32(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_pack32(t, c);
}
# endif

void bc_unpack_swap_op32(struct bc_context_s *ctx, reg_t op);
void bc_unpack_swap_op32(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
# if INT_REG_SIZE > 32 || defined(CONFIG_MUTEK_BYTECODE_VM64)
  bc_unpack32(t, c);
# endif
  bc_swap32(t, c);
}

# if INT_REG_SIZE > 32 || defined(CONFIG_MUTEK_BYTECODE_VM64)
void bc_unpack_op32(struct bc_context_s *ctx, reg_t op);
void bc_unpack_op32(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_unpack32(t, c);
}
# endif

void bc_swap_op16(struct bc_context_s *ctx, reg_t op);
void bc_swap_op16(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_swap16(t, c);
}

void bc_swap_op32(struct bc_context_s *ctx, reg_t op);
void bc_swap_op32(struct bc_context_s *ctx, reg_t op)
{
  uint_fast8_t c = op >> 4;
  bc_reg_t *t = ctx->v + (op & 15);
  bc_swap32(t, c);
}

#endif

#ifdef CONFIG_MUTEK_BYTECODE_VM

__attribute__((noinline))
static void bc_run_packing(struct bc_context_s *ctx, uint16_t op)
{
  uint_fast8_t c = ((op >> 8) & 0x7) + 1;
  uint_fast8_t r = (op & 0xf);
  bc_reg_t *t = ctx->v + r;
#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
  if (ctx->sandbox && r + c > 16)
    return;
#endif
  assert(r + c <= 16);
  op = (op >> 4) & 0xf;

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

#if (INT_REG_SIZE > 32 || defined(CONFIG_MUTEK_BYTECODE_VM64)) \
  && defined(CONFIG_MUTEK_BYTECODE_SANDBOX)
# define BC_CLAMP32(x) do { if (ctx->sandbox) (x) = (uint32_t)(x); } while (0)
#else
# define BC_CLAMP32(x) do { } while (0)
#endif

#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX

static inline uintptr_t
bc_translate_op_addr(const struct bc_descriptor_s * __restrict__ desc,
                     struct bc_context_s *ctx, bc_reg_t addr,
                     uint_fast32_t width, uint8_t nocode)
{
  if (addr & 0x80000000)    /* rw data segment */
    {
      /* address translation */
      addr &= ctx->data_addr_mask;
      addr += ctx->data_base;
    }
  else                      /* code segment */
    {
      if (nocode)
        return 0;

      if (addr + width > desc->op_count * 2)
        return 0;

      /* address translation */
      addr += (uintptr_t)desc->code;
    }

  if (addr & (width - 1))       /* not aligned */
    return 0;

  return addr;
}

uintptr_t
bc_translate_addr(struct bc_context_s *ctx, bc_reg_t addr, uint_fast32_t width)
{
  assert(ctx->sandbox);
  return bc_translate_op_addr(ctx->desc, ctx, addr, width, 0);
}
#endif

__attribute__((noinline))
static uint_fast8_t bc_run_ldst(const struct bc_descriptor_s * __restrict__ desc,
                                struct bc_context_s *ctx, const uint16_t **pc,
                                uint16_t op)
{
  dispatch_begin:;
  bc_reg_t *dst = &ctx->v[op & 0xf];
  op >>= 4;
  bc_reg_t *addrp = &ctx->v[op & 0xf];
  uintptr_t addr = *addrp;
  op >>= 4;
  uint_fast8_t inc = op & 1;
  op >>= 1;
  uint_fast8_t w = 1 << (op & 3);

  if (op & 8)
    {
      if (inc)                  /* BC_LDnE/BC_STnE */
        {
          addr += (intptr_t)(int16_t)endian_le16(*++*pc);
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

#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
  if (ctx->sandbox)
    {
      addr = bc_translate_op_addr(desc, ctx, addr, w, /* not a load */ op & 4);
      if (!addr)
        return -1;
    }
#endif

  assert((addr & (w - 1)) == 0 && "bytecode memory access not aligned");

  do {
    static const bs_dispatch_t dispatch[8] = {
      [0] = BC_DISPATCH(LD8),    [1] = BC_DISPATCH(LD16),
      [2] = BC_DISPATCH(LD32),   [3] = BC_DISPATCH(LD64),
      [4] = BC_DISPATCH(ST8),    [5] = BC_DISPATCH(ST16),
      [6] = BC_DISPATCH(ST32),   [7] = BC_DISPATCH(ST64),
    };
    BC_DISPATCH_GOTO(op & 7);

  dispatch_LD8:
    *dst = *(uint8_t*)addr;
    break;
  dispatch_LD16:
    *dst = *(uint16_t*)addr;
    break;
  dispatch_LD32:
    *dst = *(uint32_t*)addr;
    break;
  dispatch_LD64:
    *dst = *(uint64_t*)addr;
    BC_CLAMP32(*dst);
    break;
  dispatch_ST8:
    *(uint8_t*)addr = *dst;
    break;
  dispatch_ST16:
    *(uint16_t*)addr = *dst;
    break;
  dispatch_ST32:
    *(uint32_t*)addr = *dst;
    break;
  dispatch_ST64:
    *(uint64_t*)addr = *dst;
    break;
  } while (0);

  return 0;
}

__attribute__((noinline))
static bool_t bc_run_alu(struct bc_context_s *ctx, uint16_t op)
{
  dispatch_begin:;
  bc_reg_t *dstp = &ctx->v[op & 0xf];
  bc_reg_t dst = *dstp;
  uint8_t o = op;
  op >>= 4;
  o = (o ^ op) & 0xf;
  bc_reg_t src = ctx->v[op & 0xf];
  op >>= 4;

  do {
    static const bs_dispatch_t dispatch[16] = {
      [BC_OP_EQ & 0x0f] =  BC_DISPATCH(EQ),    [BC_OP_NEQ & 0x0f] = BC_DISPATCH(NEQ),
      [BC_OP_LT & 0x0f] =  BC_DISPATCH(LT),    [BC_OP_LTS & 0x0f] = BC_DISPATCH(LTS),
      [BC_OP_LTEQ & 0x0f] =  BC_DISPATCH(LTEQ),  [BC_OP_LTEQS & 0x0f] = BC_DISPATCH(LTEQS),
      [BC_OP_ADD & 0x0f] = BC_DISPATCH(ADD),   [BC_OP_SUB & 0x0f] = BC_DISPATCH(SUB),
      [BC_OP_OR & 0x0f]  = BC_DISPATCH(OR),    [BC_OP_XOR & 0x0f] = BC_DISPATCH(XOR),
      [BC_OP_AND & 0x0f] = BC_DISPATCH(AND),   [BC_OP_ANDN & 0x0f] = BC_DISPATCH(ANDN),
      [BC_OP_SHL & 0x0f] = BC_DISPATCH(SHL),   [BC_OP_SHR & 0x0f] = BC_DISPATCH(SHR),
      [BC_OP_MUL & 0x0f] = BC_DISPATCH(MUL),   [BC_OP_MOV & 0x0f] = BC_DISPATCH(MOV),
    };
    BC_DISPATCH_GOTO(op & 0x0f);

  dispatch_ADD:
    dst += src;
    BC_CLAMP32(dst);
    break;
  dispatch_SUB:
    if (!o)
      dst = 0;
    dst -= src;
    BC_CLAMP32(dst);
    break;
  dispatch_MUL:
    dst = (uint32_t)(dst * src);
    break;
  dispatch_OR:
    if (!o)
      dst = rand();
    else
      dst = (uint32_t)(dst | src);
    break;
  dispatch_XOR:
    if (o)
      dst = (uint32_t)(dst ^ src);
    else
#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
      if (!ctx->sandbox)                   /* ccall */
#endif
        ((bc_ccall_function_t*)(uintptr_t)src)(ctx);
    break;
  dispatch_AND:
    dst = (uint32_t)(dst & src);
    break;
  dispatch_ANDN:
    src = (uint32_t)~src;
    if (o)
      src &= dst;
    dst = src;
    break;
  dispatch_SHL:
    dst = (uint32_t)(dst << src);
    break;
  dispatch_SHR:
    dst = (uint32_t)(dst >> src);
    break;
  dispatch_MOV:
    dst = src;
    if (!o)
      dst = __LOG2I((uint32_t)dst);       /* msbs */
    break;
  dispatch_EQ:
  dispatch_NEQ:
    if (!o)
      src = 0;
    return (op ^ (dst != src)) & 1;
  dispatch_LT:
  dispatch_LTEQ: {
    bool_t lt = (dst < src);
    return !(lt || ((op & 4) && (dst == src)));
    }
  dispatch_LTS:
  dispatch_LTEQS: {
    bool_t lt = ((bc_sreg_t)dst < (bc_sreg_t)src);
    return !(lt || ((op & 4) && (dst == src)));
    }
  dispatch_RES:
    break;
  } while (0);

  *dstp = dst;
  return 0;
}

bc_opcode_t bc_run_vm(struct bc_context_s *ctx)
{
  const struct bc_descriptor_s * __restrict__ desc = ctx->desc;
  const uint16_t *pc = (void*)(ctx->pc & (intptr_t)-2);
  bool_t skip = ctx->pc & 1;
  uint16_t op = 0;
#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
  int_fast32_t max_cycles = ctx->max_cycles;
#endif

  if (desc->flags & BC_FLAGS_NATIVE)
    return 3;

  for (;; pc++)
    {
#if defined(CONFIG_MUTEK_BYTECODE_SANDBOX) || !defined(CONFIG_RELEASE)
      /* check pc upper bound */
      uint16_t *code_end = (uint16_t*)desc->code + desc->op_count;
      if (pc + 1 > code_end)
        goto err_pc;
#endif

      op = endian_le16(*pc);

      /* get number of extra words in the instruction */
      uint_fast8_t cst_len = 0;
      if ((op & 0xf000) == 0x7000)
        {
          cst_len += (0x1010101014121112ULL >> ((op >> 6) & 0x3c)) & 15;

#if defined(CONFIG_MUTEK_BYTECODE_SANDBOX) || !defined(CONFIG_RELEASE)
          /* check upper bound again with extra words */
          if (pc + 1 + cst_len > code_end)
            goto err_pc;
#endif
        }

      if (skip)
        {
          /* skip embedded constant value words if any */
          pc += cst_len;
          skip = 0;
          continue;
        }

#ifdef CONFIG_MUTEK_BYTECODE_TRACE
      if (ctx->trace)
        bc_dump_(ctx, pc, ctx->trace_regs);
#endif

#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
      if (max_cycles == 0)
        {
          ctx->vpc = pc;
          return 1;
        }
      max_cycles--;
#endif

      /* custom op */
      if (op & 0x8000)
        {
#ifdef CONFIG_MUTEK_BYTECODE_NATIVE
          ctx->skip = 1;
#endif
          ctx->vpc = pc + 1;
          return op;
        }

      bc_reg_t *dst = &ctx->v[op & 0xf];

      do {
	static const bs_dispatch_t dispatch[8] = {
	  [BC_OP_ADD8 >> 4] = BC_DISPATCH(add8), [BC_OP_CST8 >> 4] = BC_DISPATCH(cst8),
	  [BC_OP_JMP  >> 4]  = BC_DISPATCH(jmp), [BC_OP_LOOP >> 4] = BC_DISPATCH(loop_pack),
	  [BC_OP_FMT1 >> 4] = BC_DISPATCH(alu),  [BC_OP_FMT2 >> 4] = BC_DISPATCH(fmt2),
	  [BC_OP_LD >> 4]   = BC_DISPATCH(ldst), [BC_OP_CST >> 4] = BC_DISPATCH(cstn_call),
	};
	BC_DISPATCH_GOTO((op >> 12) & 0x7);

      dispatch_begin:
      dispatch_add8: {
          int8_t x = (op >> 4) & 0xff;
          if (x)
            {
              *dst += (bc_sreg_t)x;
              BC_CLAMP32(*dst);
              break;
            }
          if (op == 0)
            {
              ctx->vpc = pc;
              return 0;
            }
#ifdef CONFIG_MUTEK_BYTECODE_DEBUG
          else if (op == 1)
            bc_dump_(ctx, pc, 1);
#endif
#ifdef CONFIG_MUTEK_BYTECODE_TRACE
          else if (op & 8)
            {
              ctx->trace = op & 1;
              ctx->trace_regs = (op & 2) >> 1;
            }
#endif
          else if (op & 2)
            {
              if ((op & 1)      /* abort */
#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
                  && !ctx->sandbox
#endif
                  )
                abort();
              goto err_die;   /* die */
            }
          break;
        }

      dispatch_cst8:
	*dst = (bc_reg_t)((op >> 4) & 0xff);
	break;

      dispatch_jmp: {
          int8_t disp = op >> 4;
          if (disp)             /* jmp* */
            {
              if (op & 0xf)     /* jmpl */
                *dst = (bc_reg_t)(uintptr_t)pc;
              pc += (intptr_t)disp;
            }
          else                  /* ret */
            {
              pc = (void*)(uintptr_t)*dst;
            }
          goto check_pc;
        }

      dispatch_loop_pack: {
          if (op & 0x800)         /* packing */
            {
              bc_run_packing(ctx, op);
              break;
            }

          /* loop */
	  int8_t d = op >> 3;
          d >>= 1;
	  if (d < 0)
	    {
	      if (!--(*dst))
		break;
              BC_CLAMP32(*dst);
	    }
	  else if (*dst > 0)
	    {
	      (*dst)--;
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
	  if (op & 0x0800)
            {
              if (op & 0x0400)                                /* BC_EXT* */
                {
                  bc_reg_t mask = 0xffffffff >> bit;
                  bc_reg_t x = ((uint32_t)(op & 0x0200) << 22) >> bit;
                  *dst = ((*dst & mask) ^ x) - x;
                  BC_CLAMP32(*dst);
                }
              else
                *dst = op & 0x0200 ? (uint32_t)(*dst >> bit)
                                   : (uint32_t)(*dst << bit); /* BC_SHI* */
            }
          else
            {
              bc_reg_t mask = 1U << bit;
              bc_reg_t vmask = op & 0x0200 ? mask : 0;
              if (op & 0x0400)                                /* BC_BIT* */
                *dst = (*dst & ~mask) | vmask;
              else if (((*dst ^ vmask) >> bit) & 1)           /* BC_TST* */
                skip = 1;
            }
	  break;
	}

      dispatch_cstn_call: {
	  if ((op & 0x0900) == 0x0000)
	    {
              if ((op & 0x0ff0) == 0x0000) /* BC_GADDR */
                {
#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
                  if (ctx->sandbox)
                    goto err_ret;
#endif
#if INT_PTR_SIZE == 16
                  *dst = endian_16_na_load(++pc);
#endif
#if INT_PTR_SIZE == 32
                  *dst = endian_32_na_load(++pc);
#endif
#if INT_PTR_SIZE == 64
                  *dst = endian_64_na_load(++pc);
#endif
                  pc += INT_PTR_SIZE / 16 - 1;
                  break;
                }
	      uint_fast8_t c = (0x4212 >> ((op >> 7) & 0xc)) & 7;
	      bc_reg_t x = 0;
	      while (c--)
		x = (x << 16) | endian_le16(*++pc);

              BC_CLAMP32(x);

              if (op & 0x0600)  /* BC_CSTN */
                {
                  x <<= ((op & 0x00e0) >> 2);
                  if (!(op & 0x0010))
                    {
                      x = x * 2; /* BC_LADDR */
#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
                      if (!ctx->sandbox)
#endif
                        x += (uintptr_t)desc->code;
                    }
                  *dst = x;
                  break;
                }
              else             /* BC_CALL */
                {
                  if (op & 0xf)
                    *dst = (bc_reg_t)(uintptr_t)pc;
                  pc = (uint16_t*)desc->code + (bc_sreg_t)x;
                  goto check_pc;
                }
	    }
        }

      dispatch_ldst:
	if (bc_run_ldst(desc, ctx, &pc, op))
          goto err_ret;
	break;

      check_pc:
#if defined(CONFIG_MUTEK_BYTECODE_SANDBOX) || !defined(CONFIG_RELEASE)
        if (pc + 1 < (uint16_t*)desc->code || ((uintptr_t)pc & 1))
          goto err_pc;
#endif
	break;

      } while (0);

    }

 err_pc:
#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
  if (ctx->sandbox)
    goto err_ret;
#endif
  assert(!"bytecode pc out of range");

 err_die:
#ifdef CONFIG_MUTEK_BYTECODE_DEBUG
  printk("bytecode: die %p\n", pc);
  bc_dump_(ctx, pc, 1);
#endif

 err_ret:
  ctx->vpc = pc;
  return 3;
}

#endif

