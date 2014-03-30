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

#include <stdarg.h>

void bc_init_va(struct bc_context_s *ctx,
		const bc_opcode_t *code, uint_fast16_t code_sizeof,
		uint_fast8_t pcount, va_list ap)
{
  uint_fast8_t i;
  for (i = 0; i < pcount; i++)
    ctx->v[i] = va_arg(ap, uintptr_t);
  for (; i < 14; i++)
    ctx->v[i] = 0;
  if (i < 15)
    ctx->v[i] = (intptr_t)-1;
  ctx->v[15] = 0; /* pc */
  ctx->code = code;
#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
  ctx->op_count = code_sizeof / 2;
  ctx->min_addr = 0;
  ctx->max_addr = (intptr_t)-1;
  ctx->allow_ccall = 1;
#endif
#ifdef CONFIG_MUTEK_BYTECODE_TRACE
  ctx->trace = 0;
  ctx->trace_regs = 0;
#endif
}

void
bc_init(struct bc_context_s *ctx,
	const bc_opcode_t *code, uint_fast16_t code_sizeof,
	uint_fast8_t pcount, ...)
{
  va_list ap;
  va_start(ap, pcount);
  bc_init_va(ctx, code, code_sizeof, pcount, ap);
  va_end(ap);
}

#ifdef CONFIG_MUTEK_BYTECODE_DEBUG
static const char * bc_opname(uint16_t op)
{
  struct op_s
  {
    uint16_t   mask;
    uint16_t   op;
    const char *name;
  };
  static const struct op_s ops[] =
  {
    { 0x8000, 0x8000, "custom" },
    { 0xffff, 0x0000, "end" },
#ifdef CONFIG_MUTEK_BYTECODE_DEBUG
    { 0xffff, 0x0001, "dump" },
#endif
    { 0xffff, 0x0002, "abort" },
#ifdef CONFIG_MUTEK_BYTECODE_TRACE
    { 0xfffc, 0x0008, "trace" },
#endif
    { 0xf000, BC_OP_ADD8  << 8, "add8" },
    { 0xf000, BC_OP_CST8  << 8, "cst8" },
    { 0xf00f, (BC_OP_JMP  << 8) | 0xf, "jmp" },
    { 0xf000, BC_OP_JMP   << 8, "jmpl" },
    { 0xf000, BC_OP_LOOP  << 8, "loop" },
    { 0xff00, BC_OP_EQ    << 8, "eq" },
    { 0xff00, BC_OP_NEQ   << 8, "neq" },
    { 0xff00, BC_OP_LT    << 8, "lt" },
    { 0xff00, BC_OP_LTEQ  << 8, "lteq" },
    { 0xff00, BC_OP_ADD   << 8, "add" },
    { 0xff00, BC_OP_SUB   << 8, "sub" },
    { 0xff00, BC_OP_MUL   << 8, "mul" },
    { 0xff00, BC_OP_OR    << 8, "or" },
    { 0xff00, BC_OP_XOR   << 8, "xor" },
    { 0xff00, BC_OP_AND   << 8, "and" },
    { 0xff00, BC_OP_CCALL << 8, "ccall" },
    { 0xff00, BC_OP_SHL   << 8, "shl" },
    { 0xff00, BC_OP_SHR   << 8, "shr" },
    { 0xff00, BC_OP_SHRA  << 8, "shra" },
    { 0xff00, BC_OP_MOV   << 8, "mov" },
    { 0xfe00, BC_OP_TSTC  << 8, "tstc" },
    { 0xfe00, BC_OP_TSTS  << 8, "tsts" },
    { 0xfe00, BC_OP_BITC  << 8, "bitc" },
    { 0xfe00, BC_OP_BITS  << 8, "bits" },
    { 0xfc00, BC_OP_SHIL  << 8, "shil" },
    { 0xfc00, BC_OP_SHIR  << 8, "shir" },
    { 0xf900, BC_OP_LD    << 8, "ld" },
    { 0xf900, BC_OP_LDI   << 8, "ldi" },
    { 0xf900, BC_OP_ST    << 8, "st" },
    { 0xf900, BC_OP_STI   << 8, "sti" },
    { 0xff00, BC_OP_CALL  << 8, "call" },
    { 0xf900, BC_OP_CST   << 8, "cst" },
    { 0xf900, BC_OP_STD   << 8, "std" },
    { 0xf900, BC_OP_LDE   << 8, "lde" },
    { 0xf900, BC_OP_STE   << 8, "ste" },
    { 0x0000, 0x0000, "invalid" },
  };
  uint_fast8_t i;
  for (i = 0; ; i++)
    if ((op & ops[i].mask) == ops[i].op)
      return ops[i].name;
  return NULL;
}
#endif

void bc_dump(const struct bc_context_s *ctx, bool_t regs)
{
#ifdef CONFIG_MUTEK_BYTECODE_DEBUG
  uint_fast8_t i;
  uint16_t op = 0;
# ifdef CONFIG_MUTEK_BYTECODE_CHECKING
  if (ctx->v[15] < ctx->op_count)
# endif
    op = ctx->code[ctx->v[15]];

  printk("bytecode: pc=%u, opcode=%04x (%s), state before op:\n",
         ctx->v[15], op, bc_opname(op));
  if (regs)
    for (i = 0; i < 16; i++)
      printk("r%02u=%p%c", i, ctx->v[i], (i + 1) % 4 ? ' ' : '\n');
#endif
}

#define BC_DISPATCH(name) ((&&dispatch_##name - &&dispatch_begin))
#define BC_DISPATCH_GOTO(index) goto *(&&dispatch_begin + dispatch[index])
typedef int16_t bs_dispatch_t;

__attribute__((noinline))
static uint_fast8_t bc_run_ldst(struct bc_context_s *ctx, uint16_t op)
{
  dispatch_begin:;
  uintptr_t *dst = &ctx->v[op & 0xf];
  op >>= 4;
  uintptr_t *addrp = &ctx->v[op & 0xf];
  uintptr_t addr = *addrp;
  op >>= 4;
  uint_fast8_t inc = op & 1;
  op >>= 1;
  uint_fast8_t w = 1 << (op & 3);

  if (op & 8)
    {
      if (inc)                  /* BC_LDnE/BC_STnE */
        addr += (intptr_t)(int16_t)ctx->code[++ctx->v[15]];
      else                      /* BC_STnD */
        addr = (*addrp -= w);
    }
  else                          /* BC_LDn/BC_STn */
    {
      if (inc)                  /* BC_LDnI/BC_STnI */
        *addrp += w;
    }

#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
  if (addr < ctx->min_addr || addr > ctx->max_addr)
    {
# ifdef CONFIG_MUTEK_BYTECODE_DEBUG
      printk("bytecode: memory access out of range: [%p:%p] at pc=%u\n",
	     ctx->min_addr, ctx->max_addr, ctx->v[15]);
# endif
      return -1;
    }
  if (addr & (w - 1))
    {
# ifdef CONFIG_MUTEK_BYTECODE_DEBUG
      printk("bytecode: memory access not aligned at pc=%u\n", ctx->v[15]);
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
  uintptr_t *dstp = &ctx->v[op & 0xf];
  uintptr_t dst = *dstp;
  op >>= 4;
  uintptr_t src = ctx->v[op & 0xf];
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
      [BC_OP_SHRA & 0x0f] = BC_DISPATCH(SHRA), [BC_OP_MOV & 0x0f] = BC_DISPATCH(MOV),
    };
    BC_DISPATCH_GOTO(op & 0x0f);

  dispatch_BAD:
  dispatch_ADD:
    dst += src;
    break;
  dispatch_SUB:
    dst -= src;
    break;
  dispatch_MUL:
    dst *= src;
    break;
  dispatch_OR:
    dst |= src;
    break;
  dispatch_XOR:
    dst ^= src;
    break;
  dispatch_AND:
    dst &= src;
    break;
  dispatch_SHL:
    dst <<= src;
    break;
  dispatch_SHR:
    dst >>= src;
    break;
  dispatch_SHRA:
    dst = (intptr_t)dst >> src;
    break;
  dispatch_MOV:
    dst = src;
    break;
  dispatch_EQ:
  dispatch_NEQ:
    if ((op ^ (dst != src)) & 1)
      ctx->v[15]++;
    break;
  dispatch_LT:
  dispatch_LTEQ:
    if (!((dst < src) | (op & 1 & (dst == src))))
      ctx->v[15]++;
    break;
  dispatch_CCALL:
#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
    if (!ctx->allow_ccall)
      {
# ifdef CONFIG_MUTEK_BYTECODE_DEBUG
        printk("bytecode: C function call is not allowed at pc=%u\n", ctx->v[15]);
# endif
        return -1;
      }
#endif
    dst = ((bc_ccall_function_t*)src)(ctx, dst);
  dispatch_RES:
    break;
  } while (0);

  *dstp = dst;
  return 0;
}

uint16_t bc_run(struct bc_context_s *ctx, int_fast32_t max_cycles)
{
  uint16_t op = 0;

  for (;; ctx->v[15]++)
    {
#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
      if (ctx->v[15] >= ctx->op_count)
	goto err_pc;
#endif

      op = ctx->code[ctx->v[15]];

#ifdef CONFIG_MUTEK_BYTECODE_TRACE
      if (ctx->trace)
        bc_dump(ctx, ctx->trace_regs);
#endif

#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
      if (max_cycles == 0)
	return 1;
      if (max_cycles > 0)
	max_cycles--;
#endif

      /* custom op */
      if (op & 0x8000)
        return op;

      uintptr_t *dst = &ctx->v[op & 0xf];

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
	if (op == BC_END())
	  return op;
#ifdef CONFIG_MUTEK_BYTECODE_DEBUG
	else if (op == BC_DUMP())
	  bc_dump(ctx, 1);
#endif
#ifdef CONFIG_MUTEK_BYTECODE_TRACE
	else if ((op & 0xfffc) == 8)
          {
            ctx->trace = op & 1;
            ctx->trace_regs = (op & 2) >> 1;
          }
#endif
	else if (op == BC_ABORT())
	  goto err_abort;
        else
          *dst += (intptr_t)(int8_t)((op >> 4) & 0xff);
	break;

      dispatch_cst8:
	*dst = (uintptr_t)((op >> 4) & 0xff);
	break;

      dispatch_jmp: {
        uintptr_t pc = ctx->v[15];
        *dst = pc;
	ctx->v[15] = pc + (int8_t)(op >> 4);
	break;
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
	  ctx->v[15] += d;
	  break;
	}

      dispatch_alu:
	if (bc_run_alu(ctx, op))
          return op;
	break;

      dispatch_fmt2: {
	  uint_fast8_t bit = (op >> 4) & 0x3f;
	  if (op & 0x0800)
	    *dst = op & 0x0400 ? *dst >> bit : *dst << bit; /* BC_SHI* */
	  else {
            bit &= 0x1f;
            uintptr_t mask = 1U << bit;
            uintptr_t vmask = op & 0x0200 ? mask : 0;
            if (op & 0x0400)                                /* BC_BIT* */
              *dst = (*dst & ~mask) | vmask;
	    else if (((*dst ^ vmask) >> bit) & 1)           /* BC_TST* */
	      ctx->v[15]++;
          }
	  break;
	}

      dispatch_cstn_call: {
	  if ((op & 0x0900) == 0x0000)
	    {
	      uint_fast8_t c = (0x4212 >> ((op >> 7) & 0xc)) & 7;
	      uintptr_t x = 0;
#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
	      if (ctx->v[15] + c >= ctx->op_count)
		goto err_pc;
#endif
	      while (c--)
		x = (x << 16) | ctx->code[++ctx->v[15]];
	      x <<= ((op & 0x00e0) >> 2);
              if (op & 0x0600)  /* BC_CSTN */
                {
                  if (op & 0x0010)
                    x = ~x;
                  *dst = x;
                }
	      else              /* BC_CALL */
                {
                  *dst = ctx->v[15];
                  ctx->v[15] = x;
                }
	      break;
	    }
        }

      dispatch_ldst:
	if (bc_run_ldst(ctx, op))
	  return op;
	break;

      } while (0);

    }

#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
 err_pc:
# ifdef CONFIG_MUTEK_BYTECODE_DEBUG
  printk("bytecode: pc out of range: pc=%u\n", ctx->v[15]);
# endif
  return op;

 err_loop:
# ifdef CONFIG_MUTEK_BYTECODE_DEBUG
  printk("bytecode: loop index is zero: pc=%u\n", ctx->v[15]);
# endif
  return op;

#endif
 err_abort:
#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
  printk("bytecode: abort\n", ctx->v[15]);
# ifdef CONFIG_MUTEK_BYTECODE_DEBUG
  bc_dump(ctx, 1);
# endif
#endif
  return op;
}
