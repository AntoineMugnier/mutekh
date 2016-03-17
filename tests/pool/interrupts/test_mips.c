
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
int syscall();
void end_test();

asm(
".text \n"
"fault1:"
    ".set push \n"
    ".set noreorder \n"
    ".set noat \n"
    "addiu    $2, $0, 1 \n"
    "lw       $1, 1($0) \n" // bad align
    "addiu    $2, $2, 41 \n"
    "jr       $31 \n"
    "nop \n" // return stack pointer
"fault2:"
    "addiu    $2, $0, 1 \n"
    "b        1f \n"
    "lw       $1, 1($0) \n" // bad align in branch delay slot
    "addiu    $2, $2, 1 \n"
"1: \n"
    "addiu    $2, $2, 42 \n"
    "jr       $31 \n"
    "nop \n"
"fault3:"
    "lw       $1, 1($0) \n" // bad align
    "jr       $31 \n"
    "move     $2, $sp \n" // return stack pointer
    ".set pop \n"
);

static CPU_EXCEPTION_HANDLER(fault_handler)
{

  switch (step++)
    {
    case 0:
      if (type == CPU_EXCEPTION_ADDR_LOAD &&
          execptr == (uintptr_t)&fault1 + 4)
        ok |= 1;
      return;

    case 1:
      if (execptr == (uintptr_t)&fault1 + 4)
        ok |= 2;
      cpu_exception_resume_pc(regs, (uintptr_t)&fault1 + 8);
      return;

    case 2:
      if (execptr == (uintptr_t)&fault2 + 8)
        ok |= 8;
      return;

    case 3:
      if (execptr == (uintptr_t)&fault2 + 8)
        ok |= 16;
      cpu_exception_resume_pc(regs, (uintptr_t)&fault2 + 16);
      return;

    case 4:
      if (execptr == (uintptr_t)&fault3)
        stack = stackptr;
      cpu_exception_resume_pc(regs, (uintptr_t)&fault3 + 4);
      return;

#ifdef CONFIG_HEXO_USERMODE
    case 8:
      if (type == CPU_EXCEPTION_COPROC &&
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
    ".set push \n"
    ".set noreorder \n"
    ".set noat \n"
"syscall:"
    "addiu   $4, 17 \n" // change input arg
    "syscall \n"
    "addiu   $4, 7 \n"
    "addiu   $4, 5 \n"
    "syscall \n"
    "addiu   $4, 3 \n"     // skipped
    "addiu   $4, 8 \n"
    "addiu   $sp, $sp, 33 \n" // bad user stack
    "syscall \n"
    "nop \n"
    "mtc0    $0, $12 \n" // user fault
    "addiu   $4, $2, 11 \n" 
    "addiu   $27, $0, 3\n" // destroy cls
    "syscall \n"
    ".set pop \n"
);

static CPU_SYSCALL_HANDLER(syscall_handler)
{
  printk("syscall %i\n", regs->gpr[4]);

  switch (step++)
    {
      case 5:
        if (number == 0 && regs->gpr[4] == 351+17) // syscall arg
          ok |= 64;
        return;

      case 6:
        if (number == 0 && regs->gpr[4] == 351+17+7+5) // syscall arg
          ok |= 128;
        cpu_exception_resume_pc(regs, (uintptr_t)&syscall + 24);
        return;

      case 7:
        if (number == 0 && regs->gpr[4] == 351+17+7+5+8) // syscall arg
          ok |= 256;
        regs->gpr[2] = 117;
        return;

      case 9:
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
  if (fault2() == 43)
    ok |= 32;

  if (fault3() == stack)
    ok |= 2048;

#ifdef CONFIG_HEXO_USERMODE
  cpu_syscall_sethandler(syscall_handler);
  cpu_context_set_user(1234 + 16, (uintptr_t)&syscall, 351);
#else
  ok |= 64 | 128 | 256 | 512 | 1024;
#endif

  end_test();
}

void end_test()
{
  if (ok == 4095)
    printk("++SUCCESS++%i++\n", ok);
  else
    printk("++FAIL++%x++\n", ok);

  power_shutdown();
  power_reboot();
}

