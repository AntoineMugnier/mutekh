
#include <mutek/printk.h>
#include <hexo/context.h>
#include <hexo/interrupt.h>
#include <hexo/power.h>

volatile int ok = 0;
volatile int step = 0;
volatile uintptr_t stack;

int fault1();
int fault2();
uintptr_t fault3();
void end_test();

asm(
".text \n"
"fault1:"
    "addi     r1,  r0,  1 \n"
    "divu     r2,  r2,  r0 \n"
    "addi     r1,  r1, 41 \n"
    "b        ra          \n"
"fault2:"
    "addi     r1,  r0,  1 \n"
    "divu     r2,  r2,  r0 \n"
    "addi     r1,  r1, 41 \n"
    "addi     r1,  r1, 17 \n"
    "b        ra          \n"
"fault3:"
    "divu     r2,  r2,  r0 \n"
    "mv       r1,  sp     \n"
    "b        ra          \n"
);

static CPU_EXCEPTION_HANDLER(fault_handler)
{

  switch (step++)
    {
    case 0:
      if (type == CPU_FAULT_DIV_BY_0 &&
          execptr == (uintptr_t)&fault1 + 4)
        ok |= 1;
      return;

    case 1:
      if (type == CPU_FAULT_DIV_BY_0 &&
          execptr == (uintptr_t)&fault1 + 4)
        ok |= 2;
      cpu_exception_resume_pc(regs, (uintptr_t)&fault1 + 8);
      return;

    case 2:
      if (execptr == (uintptr_t)&fault2 + 4)
        ok |= 8;
      cpu_exception_resume_pc(regs, (uintptr_t)&fault2 + 12);
      return;

    case 3:
      if (execptr == (uintptr_t)&fault3)
        stack = stackptr;
      cpu_exception_resume_pc(regs, (uintptr_t)&fault3 + 4);
      return;
    }
}

void main()
{
  cpu_exception_sethandler(fault_handler);

  if (fault1() == 42)
    ok |= 4;
  if (fault2() == 18)
    ok |= 16;
  if (fault3() == stack)
    ok |= 32;

  end_test();
}

void end_test()
{
  if (ok == 63)
    printk("++SUCCESS++%i++\n", ok);
  else
    printk("++FAIL++%x++\n", ok);

  power_shutdown();
  power_reboot();
}

