
#include <mutek/printk.h>
#include <hexo/context.h>
#include <hexo/interrupt.h>
#include <hexo/power.h>
#include <hexo/cpu.h>

volatile int ok = 0;
volatile int step = 0;
volatile uintptr_t stack;

int fault1();
int fault2();
uintptr_t fault3();
int syscall();
void end_test();

asm(
".text \n"
"fault1:"
    ".set noat \n"
    "addi     r4,  r0,  1 \n"
    "ldw      r1,  1(r0)  \n" // bad align
    "addi     r2,  r4, 41 \n"
    "ret                  \n"
"fault2:"
    "addi     r4,  r0,  1 \n"
    "ldw      r1,  1(r0)  \n" // bad align
    "addi     r4, r4, 41  \n"
    "addi     r2, r4, 17  \n"
    "ret                  \n"
"fault3:" 
    "ldw      r1,  1(r0)  \n" // bad align
    "mov      r2, sp      \n"
    "ret                  \n"
    ".set at \n"
);

static CPU_EXCEPTION_HANDLER(fault_handler)
{
  printk("fault %i\n", type);

  switch (step++)
    {
    case 0:
      if (type == CPU_EXCEPTION_MISALIGNED_DATA &&
          execptr == (uintptr_t)&fault1 + 4)
        ok |= 1;
      return;

    case 1:
      if (type == CPU_EXCEPTION_MISALIGNED_DATA &&
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

#ifdef CONFIG_HEXO_USERMODE
    case 7:
      if (type == CPU_EXCEPTION_SUPERVISOR_INS &&
          execptr == (uintptr_t)&syscall + 40 &&
          stackptr == 1234+33)
        ok |= 512;
      cpu_exception_resume_pc(regs, (uintptr_t)&syscall + 44);
      return;
#endif
    }
}

#ifdef CONFIG_HEXO_USERMODE

asm(
".text \n"
"syscall:"
    ".set noat \n"
    "addi    r4, r4, 17 \n"
    "trap               \n"
    "addi    r4, r4, 7  \n"
    "addi    r4, r4, 5  \n"
    "trap               \n"
    "addi    r4, r4, 3  \n"     // skipped
    "addi    r4, r4, 8  \n"
    "addi    sp, sp, 33 \n" // bad user stack
    "trap               \n"
    "movi    " ASM_STR(CPU_NIOS2_CLS_REG) ", 0  \n" // kill cls reg
    "bret               \n" // user fault
    "addi    r4, r4, 11  \n"
    "trap               \n"
    ".set at \n"
);

static CPU_SYSCALL_HANDLER(syscall_handler)
{
  printk("syscall %i\n", regs->gpr[4]);

  switch (step++)
    {
      case 4:
        if (number == 0 && regs->gpr[4] == 351+17) // syscall arg
          ok |= 64;
        return;

      case 5:
        if (number == 0 && regs->gpr[4] == 351+17+7+5) // syscall arg
          ok |= 128;
        cpu_exception_resume_pc(regs, (uintptr_t)&syscall + 24);
        return;

      case 6:
        if (number == 0 && regs->gpr[4] == 351+17+7+5+8) // syscall arg
          ok |= 256;
        regs->gpr[4] = 117;
        return;

      case 8:
        if (number == 0 && regs->gpr[4] == 117+11) // syscall arg
          ok |= 1024;
        end_test();
    }

}
#endif

void main()
{
  cpu_exception_sethandler(fault_handler);

  if (fault1() == 42)
    ok |= 4;
  if (fault2() == 18)
    ok |= 16;

  if (fault3() == stack)
    ok |= 32;

#ifdef CONFIG_HEXO_USERMODE
  cpu_syscall_sethandler(syscall_handler);
  cpu_context_set_user(1234, (uintptr_t)&syscall, 351);
#else
  ok |= 64 | 128 | 256 | 512 | 1024;
#endif

  end_test();
}

void end_test()
{
  if (ok == 2047)
    printk("++SUCCESS++%i++\n", ok);
  else
    printk("++FAIL++%x++\n", ok);

  power_shutdown();
  power_reboot();
}

