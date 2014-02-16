
#include <mutek/printk.h>
#include <mutek/bytecode.h>

#define BC_CUSTOM_PRINTI(r) \
  BC_CUSTOM(0x1000 | r)
#define BC_CUSTOM_PRINTS(r) \
  BC_CUSTOM(0x2000 | r)

static BC_CCALL_FUNCTION(c_func)
{
    return dst * 13;
}

void app_start()
{
  static const bc_opcode_t test[] = {

    /* CST8, CCALL, NEQ */
    BC_CST8(2, 11),
    BC_CCALL(2, 1),
    BC_CST8(3, 143),
    BC_NEQ(3, 2),
    BC_ABORT(),

    /* CST8, XOR, NEQ, EQ */
    BC_CST8(1, 0),
    BC_XOR(2, 2),
    BC_NEQ(1, 2),
    BC_ABORT(),
    BC_EQ(1, 2),
    BC_JMP(1),
    BC_ABORT(),

    /* CST8, EQ, ADD */
    BC_CST8(1, 42),
    BC_EQ(1, 2),
    BC_ABORT(),
    BC_NEQ(1, 2),
    BC_JMP(1),
    BC_ABORT(),
    BC_ADD8(2, 21),
    BC_ADD8(2, 21),
    BC_NEQ(1, 2),
    BC_ABORT(),

    /* LT, LTEQ */
    BC_LT(1, 2),
    BC_ABORT(),
    BC_LTEQ(1, 2),
    BC_JMP(1),
    BC_ABORT(),
    BC_ADD8(1, 1),
    BC_LT(1, 2),
    BC_ABORT(),
    BC_LTEQ(1, 2),
    BC_ABORT(),
    BC_LT(2, 1),
    BC_JMP(1),
    BC_ABORT(),
    BC_LTEQ(2, 1),
    BC_JMP(1),
    BC_ABORT(),

    /* LOOP backward, ADD8 positiv */
    BC_CST8(1, 0),
    BC_CST8(2, 5),
  /* label:add */
    BC_ADD8(1, 2),
    BC_LOOP(2, -2 /* :add */),
    BC_CST8(2, 10),
    BC_NEQ(1, 2),
    BC_ABORT(),
    
    /* ADD8 negative */
    BC_CST8(1, 15),
    BC_CST8(2, 5),
  /* label:add8 */
    BC_ADD8(1, -2),
    BC_LOOP(2, -2 /* :add8 */),
    BC_CST8(2, 5),
    BC_NEQ(1, 2),
    BC_ABORT(),

    /* LOOP forward, JMP */
    BC_CST8(1, 0),
    BC_CST8(2, 5),
  /* label:re */
    BC_LOOP(2, 2 /* :fwd */),
    BC_ADD8(1, 2),
    BC_JMP(-3 /* :re */),
  /* label:fwd */
    BC_CST8(2, 10),
    BC_NEQ(1, 2),
    BC_ABORT(),

    /* TSTC, TSTS */
    BC_CST8(1, 0x55),
    BC_TSTC(1, 2),
    BC_ABORT(),
    BC_TSTS(1, 2),
    BC_JMP(1),
    BC_ABORT(),
    BC_TSTS(1, 1),
    BC_ABORT(),
    BC_TSTC(1, 1),
    BC_JMP(1),
    BC_ABORT(),
    BC_TSTC(1, 1),
    BC_ADD(1, 1),
    BC_TSTC(1, 1),
    BC_ABORT(),
    BC_TSTS(1, 1),
    BC_JMP(1),
    BC_ABORT(),
    BC_TSTS(1, 2),
    BC_ABORT(),
    BC_TSTC(1, 2),
    BC_JMP(1),
    BC_ABORT(),

    /* BITC, BITS */
    BC_CST8(1, 0x0f),
    BC_BITS(1, 5),
    BC_TSTC(1, 5),
    BC_ABORT(),
    BC_BITC(1, 2),
    BC_TSTS(1, 2),
    BC_ABORT(),
    BC_CST8(2, 0x2b),
    BC_NEQ(1, 2),
    BC_ABORT(),

    /* LD8, LD8I */
    BC_MOV(1, 0),
    BC_ADD8(1, 5),
    BC_MOV(4, 1),
    BC_LD8(2, 1),
    BC_CST8(3, 'a'),
    BC_NEQ(3, 2),
    BC_ABORT(),
    BC_NEQ(4, 1),
    BC_ABORT(),
    BC_LD8I(2, 1),
    BC_LD8I(2, 1),
    BC_CST8(3, 'r'),
    BC_NEQ(3, 2),
    BC_ABORT(),

    /* MUL, CST16, CST32, CST16X */
    BC_CST16(1, 0x1234),
    BC_CST32(2, 0x5678abcd),
    BC_MUL(2, 1),
    BC_CST16X(3, 0xffff, 16, 0),
    BC_AND(2, 3),
    BC_CST32(3, 0x0c970000),
    BC_NEQ(3, 2),
    BC_ABORT(),

    /* ST32I, ST32, LD32 */
    BC_CST32(2, 0x12345678),
    BC_MOV(1, 0),
    BC_ST32I(2, 1),
    BC_ADD(2, 2),
    BC_ST32(2, 1),
    BC_LD32(3, 1),
    BC_CST32(2, 0x2468acf0),
    BC_NEQ(3, 2),
    BC_ABORT(),
    BC_ADD8(1, -4),
    BC_NEQ(1, 0),
    BC_ABORT(),
    BC_LD32(4, 1),
    BC_CST32(2, 0x12345678),
    BC_NEQ(4, 2),
    BC_ABORT(),

    /* hello world using custom instruction */
    BC_MOV(2, 0),
    BC_MOV(1, 0),
    BC_CST8(5, 'H'),
    BC_ST8I(5, 1),
    BC_CST8(5, 'e'),
    BC_ST8I(5, 1),
    BC_CST8(5, 'l'),
    BC_ST8I(5, 1),
    BC_ST8I(5, 1),
    BC_CST8(5, 'o'),
    BC_ST8I(5, 1),
    BC_CST8(5, 0),
    BC_ST8I(5, 1),

    BC_CUSTOM_PRINTS(2),

    /* recursive factorial invocation */
    BC_MOV(13, 0),              /* setup stack ptr */
    BC_ADD8(13, 120),
    BC_CST8(1, 5),              /* param */
    BC_CALL(10, 165 /* :fact */),
    BC_CST8(3, 120),            /* test return value */
    BC_NEQ(1, 3),
    BC_ABORT(),
    BC_END(),

    /* recursive factorial function */
  /* label:fact */
    BC_ST16D(10, 13),
    BC_CST8(2, 1),   
    BC_EQ(1, 2),     
    BC_JMP(5 /* :fact_end */),
    BC_ST16D(1, 13), 
    BC_ADD8(1, -1),  
    BC_JMPL(10, -7 /* :fact */),  /* call */
    BC_LD16I(2, 13), 
    BC_MUL(1, 2),    
  /* label:fact_end */
    BC_CUSTOM_PRINTI(1),
    BC_LD16I(10, 13),
    BC_MOV(15, 10),
  };

  struct bc_context_s vm;
  char buf[128] = "testbarx";

  bc_init(&vm, test, sizeof(test), 2, buf, &c_func);
  bc_set_addr_range(&vm, (uintptr_t)buf, (uintptr_t)buf + sizeof(buf) - 1);
  //  bc_dump(&vm);

  while (1)
    {
      uint16_t r = bc_run(&vm, -1);

      if (!r)
        {
          printk("done\n");
          return;  /* terminate */
        }
      if (!(r & 0x8000))
        {
          printk("bytecode execution error\n");
          return; /* error */
        }

      switch (r & 0xf000)
        {
        case BC_CUSTOM_PRINTI(0):
          printk("%u\n", bc_get_reg(&vm, r & 0xf));
          break;
        case BC_CUSTOM_PRINTS(0):
          printk("%s\n", (char*)bc_get_reg(&vm, r & 0xf));
          break;
        default:
          printk("bad custom bytecode opcode\n");
          return;
        }
      bc_skip(&vm);
    }

  while (1)
   ;
}
