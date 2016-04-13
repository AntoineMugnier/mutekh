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

void
bc_init(struct bc_context_s *ctx,
        const struct bc_descriptor_s *desc)
{
  ctx->pc = desc->code;
  ctx->skip = 2;
  ctx->desc = desc;
#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
  ctx->min_addr = 0;
  ctx->max_addr = (intptr_t)-1LL;
  ctx->sandbox = 0;
#endif
#ifdef CONFIG_MUTEK_BYTECODE_TRACE
  ctx->trace = 0;
  ctx->trace_regs = 0;
#endif
}

#ifdef CONFIG_MUTEK_BYTECODE_VM

#ifdef CONFIG_MUTEK_BYTECODE_DEBUG
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
    { 0xf000, BC_OP_LOOP  << 8, "loop" },
    { 0xff00, BC_OP_EQ    << 8, "eq", "eq0" },
    { 0xff00, BC_OP_NEQ   << 8, "neq", "neq0" },
    { 0xff00, BC_OP_LT    << 8, "lt" },
    { 0xff00, BC_OP_LTEQ  << 8, "lteq" },
    { 0xff00, BC_OP_ADD   << 8, "add" },
    { 0xff00, BC_OP_SUB   << 8, "sub", "neg" },
    { 0xff00, BC_OP_MUL   << 8, "mul" },
    { 0xff00, BC_OP_OR    << 8, "or" },
    { 0xff00, BC_OP_XOR   << 8, "xor" },
    { 0xff00, BC_OP_AND   << 8, "and" },
    { 0xff00, BC_OP_CCALL << 8, "ccall" },
    { 0xff00, BC_OP_SHL   << 8, "shl" },
    { 0xff00, BC_OP_SHR   << 8, "shr" },
    { 0xff00, BC_OP_ANDN  << 8, "andn", "not" },
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

void bc_dump(const struct bc_context_s *ctx, bool_t regs)
{
#ifdef CONFIG_MUTEK_BYTECODE_DEBUG
  uint_fast8_t i;

  if (ctx->desc->flags & BC_FLAGS_NATIVE)
    {
      printk("bytecode: pc=%p", ctx->pc);
    }
  else
    {
      printk("bytecode: pc=%p (%u)", ctx->pc, ctx->pc - (uint16_t*)ctx->desc->code);

# ifdef CONFIG_MUTEK_BYTECODE_CHECKING
      if (ctx->pc >= (uint16_t*)ctx->desc->code &&
          ctx->pc < (uint16_t*)ctx->desc->code + ctx->desc->op_count &&
          !((uintptr_t)ctx->pc & 1))
# endif
        {
          uint16_t op = endian_le16(*ctx->pc);

          printk(", opcode=%04x (%s)", op, bc_opname(op));
        }
    }

  printk("\n");

  if (regs)
    for (i = 0; i < 16; i++)
      printk("r%02u=%" BC_REG_FORMAT "%c", i, ctx->v[i], (i + 1) % 4 ? ' ' : '\n');
#endif
}

#define BC_DISPATCH(name) ((&&dispatch_##name - &&dispatch_begin))
#define BC_DISPATCH_GOTO(index) goto *(&&dispatch_begin + dispatch[index])
typedef int16_t bs_dispatch_t;

__attribute__((noinline))
static uint_fast8_t bc_run_ldst(const struct bc_descriptor_s * __restrict__ desc,
                                struct bc_context_s *ctx, uint16_t op)
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
        addr += (intptr_t)(int16_t)endian_le16(*++ctx->pc);
      else                      /* BC_STnD */
        addr = (*addrp -= w);
    }
  else                          /* BC_LDn/BC_STn */
    {
      if (inc)                  /* BC_LDnI/BC_STnI */
        *addrp += w;
    }

#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
  if ((addr < ctx->min_addr || addr > ctx->max_addr) && /* out of user range */
      ((op & 4) ||                                      /* not a load */
       (void*)addr < desc->code ||                 /* out of bytecode */
       (uint16_t*)addr >= (uint16_t*)desc->code + desc->op_count))
    {
# ifdef CONFIG_MUTEK_BYTECODE_DEBUG
      printk("bytecode: memory access out of range: [%p:%p] at pc=%p\n",
	     ctx->min_addr, ctx->max_addr, ctx->pc);
# endif
      return -1;
    }
  if (addr & (w - 1))
    {
# ifdef CONFIG_MUTEK_BYTECODE_DEBUG
      printk("bytecode: memory access not aligned at pc=%p\n", ctx->pc);
# endif
      return -1;
    }
#endif

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
static uint_fast8_t bc_run_alu(struct bc_context_s *ctx, uint16_t op)
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
      [BC_OP_LT & 0x0f] =  BC_DISPATCH(LT),    [BC_OP_LTEQ & 0x0f] = BC_DISPATCH(LTEQ),
      [BC_OP_ADD & 0x0f] = BC_DISPATCH(ADD),   [BC_OP_SUB & 0x0f] = BC_DISPATCH(SUB),
      [BC_OP_RES & 0x0f] = BC_DISPATCH(RES),   [BC_OP_MUL & 0x0f] = BC_DISPATCH(MUL),
      [BC_OP_OR & 0x0f]  = BC_DISPATCH(OR),    [BC_OP_XOR & 0x0f] = BC_DISPATCH(XOR),
      [BC_OP_AND & 0x0f] = BC_DISPATCH(AND),   [BC_OP_CCALL & 0x0f] = BC_DISPATCH(CCALL),
      [BC_OP_SHL & 0x0f] = BC_DISPATCH(SHL),   [BC_OP_SHR & 0x0f] = BC_DISPATCH(SHR),
      [BC_OP_ANDN & 0x0f] = BC_DISPATCH(ANDN),  [BC_OP_MOV & 0x0f] = BC_DISPATCH(MOV),
    };
    BC_DISPATCH_GOTO(op & 0x0f);

  dispatch_BAD:
  dispatch_ADD:
    dst += src;
    break;
  dispatch_SUB:
    if (!o)
      dst = 0;
    dst -= src;
    break;
  dispatch_MUL:
    dst = (uint32_t)(dst * src);
    break;
  dispatch_OR:
    dst = (uint32_t)(dst | src);
    break;
  dispatch_XOR:
    dst = (uint32_t)(dst ^ src);
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
    if ((op ^ (dst != src)) & 1)
      ctx->pc++;
    break;
  dispatch_LT:
  dispatch_LTEQ:
    if (!((dst < src) | (op & 1 & (dst == src))))
      ctx->pc++;
    break;
  dispatch_CCALL:
#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
    if (ctx->sandbox)
      {
# ifdef CONFIG_MUTEK_BYTECODE_DEBUG
        printk("bytecode: C function call is not allowed at pc=%p\n", ctx->pc);
# endif
        return -1;
      }
#endif
    dst = ((bc_ccall_function_t*)(uintptr_t)src)(ctx, dst);
  dispatch_RES:
    break;
  } while (0);

  *dstp = dst;
  return 0;
}

bc_opcode_t bc_run_vm(struct bc_context_s *ctx, int_fast32_t max_cycles)
{
  const struct bc_descriptor_s * __restrict__ desc = ctx->desc;
  uint16_t op = 0;

  if (desc->flags & BC_FLAGS_NATIVE)
    return 3;

  for (;; ctx->pc++)
    {
      op = endian_le16(*ctx->pc);

#ifdef CONFIG_MUTEK_BYTECODE_TRACE
      if (ctx->trace)
        bc_dump(ctx, ctx->trace_regs);
#endif

#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
      if (max_cycles == 0)
	return 1;
      max_cycles--;
#endif

      /* custom op */
      if (op & 0x8000)
        {
          ctx->pc++;
          return op;
        }

      bc_reg_t *dst = &ctx->v[op & 0xf];

      do {
	static const bs_dispatch_t dispatch[8] = {
	  [BC_OP_ADD8 >> 4] = BC_DISPATCH(add8), [BC_OP_CST8 >> 4] = BC_DISPATCH(cst8),
	  [BC_OP_JMP  >> 4]  = BC_DISPATCH(jmp), [BC_OP_LOOP >> 4] = BC_DISPATCH(loop),
	  [BC_OP_FMT1 >> 4] = BC_DISPATCH(alu),  [BC_OP_FMT2 >> 4] = BC_DISPATCH(fmt2),
	  [BC_OP_LD >> 4]   = BC_DISPATCH(ldst), [BC_OP_CST >> 4] = BC_DISPATCH(cstn_call),
	};
	BC_DISPATCH_GOTO((op >> 12) & 0x7);

      dispatch_begin:
      dispatch_add8:
	if (op == 0)
          return 0;
#ifdef CONFIG_MUTEK_BYTECODE_DEBUG
	else if (op == 1)
	  bc_dump(ctx, 1);
#endif
#ifdef CONFIG_MUTEK_BYTECODE_TRACE
	else if ((op & 0xfffc) == 8)
          {
            ctx->trace = op & 1;
            ctx->trace_regs = (op & 2) >> 1;
          }
#endif
	else if (op == 2)
	  goto err_abort;
        else
          *dst += (bc_sreg_t)(int8_t)((op >> 4) & 0xff);
	break;

      dispatch_cst8:
	*dst = (bc_reg_t)((op >> 4) & 0xff);
	break;

      dispatch_jmp: {
          int8_t disp = op >> 4;
          if (disp)             /* jmp* */
            {
              if (op & 0xf)     /* jmpl */
                *dst = (bc_reg_t)(uintptr_t)ctx->pc;
              ctx->pc += (intptr_t)disp;
            }
          else                  /* ret */
            {
              ctx->pc = (void*)(uintptr_t)*dst;
            }
          goto check_pc;
        }

      dispatch_loop: {
	  int8_t d = op >> 4;
	  if (d < 0)
	    {
#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
	      if (*dst == 0)
		goto err_loop;
#endif
	      if (!--(*dst))
		break;
	    }
	  else if (*dst > 0)
	    {
	      (*dst)--;
	      break;
	    }
	  ctx->pc += d;
          goto check_pc;
	}

      dispatch_alu:
	if (bc_run_alu(ctx, op))
          return 3;
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
                ctx->pc++;
            }
	  break;
	}

      dispatch_cstn_call: {
	  if ((op & 0x0900) == 0x0000)
	    {
              if ((op & 0x0ff0) == 0x0000) /* BC_GADDR */
                {
#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
                  if (ctx->sandbox)
                    goto err_pc;
#endif
#if INT_PTR_SIZE == 16
                  *dst = endian_16_na_load(++ctx->pc);
#endif
#if INT_PTR_SIZE == 32
                  *dst = endian_32_na_load(++ctx->pc);
#endif
#if INT_PTR_SIZE == 64
                  *dst = endian_64_na_load(++ctx->pc);
#endif
                  ctx->pc += INT_PTR_SIZE / 16 - 1;
                  break;
                }
	      uint_fast8_t c = (0x4212 >> ((op >> 7) & 0xc)) & 7;
	      bc_reg_t x = 0;
#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
	      if ((uint16_t*)desc->code + desc->op_count - c < ctx->pc)
		goto err_pc;
#endif
	      while (c--)
		x = (x << 16) | endian_le16(*++ctx->pc);
              if (op & 0x0600)  /* BC_CSTN */
                {
                  x <<= ((op & 0x00e0) >> 2);
                  if (!(op & 0x0010))
                    x = x * 2 + (uintptr_t)desc->code; /* BC_LADDR */
                  *dst = x;
                  break;
                }
              else             /* BC_CALL */
                {
                  if (op & 0xf)
                    *dst = (bc_reg_t)(uintptr_t)ctx->pc;
                  ctx->pc = (uint16_t*)desc->code + x;
                  goto check_pc;
                }
	    }
        }

      dispatch_ldst:
	if (bc_run_ldst(desc, ctx, op))
	  return 3;
	break;

      check_pc:
#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
        if (ctx->pc < (uint16_t*)desc->code ||
            ctx->pc >= (uint16_t*)desc->code + desc->op_count ||
            ((uintptr_t)ctx->pc & 1))
          goto err_pc;
#endif
	break;

      } while (0);

    }

#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
 err_pc:
# ifdef CONFIG_MUTEK_BYTECODE_DEBUG
  printk("bytecode: pc out of range: pc=%p\n", ctx->pc);
# endif
  return 3;

 err_loop:
# ifdef CONFIG_MUTEK_BYTECODE_DEBUG
  printk("bytecode: loop index is zero: pc=%p\n", ctx->pc);
# endif
  return 3;

#endif
 err_abort:
#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
  printk("bytecode: abort %p\n", ctx->pc);
# ifdef CONFIG_MUTEK_BYTECODE_DEBUG
  bc_dump(ctx, 1);
# endif
#endif
  return 3;
}

#endif

