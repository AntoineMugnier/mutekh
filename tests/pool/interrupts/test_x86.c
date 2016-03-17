
#include <mutek/printk.h>
#include <hexo/context.h>
#include <hexo/interrupt.h>
#include <hexo/power.h>

volatile int ok = 0;
volatile int step = 0;
volatile uintptr_t stack;

int fault1();
int fault1_ud();
int fault1_resume();
int fault2();
int fault2_ud();
int fault2_resume();
uintptr_t fault3();
uintptr_t fault3_resume();
void end_test();

void syscall();
void syscall_resume();
void syscall_fault();
void syscall_resume2();

asm(
".text \n"
"fault1:"
    "mov      $1, %eax \n"
"fault1_ud:"
    "ud2 \n" // bad opcode
"fault1_resume:"
    "add      $41, %eax \n"
    "ret \n"
"fault2:"
    "mov      $1, %eax \n"
"fault2_ud:"
    "jmpl     $0, $1f \n"
    "add      $1, %eax \n"
"fault2_resume:"
"1: \n"
    "add      $42, %eax \n"
    "ret \n"
"fault3:"
    "ud2 \n" // bad opcode
"fault3_resume:"
    "mov      %esp, %eax \n" // return stack pointer
    "ret \n"
);

static CPU_EXCEPTION_HANDLER(fault_handler)
{

  switch (step++)
    {
    case 0:
      if (type == CPU_EXCEPTION_INVALID_OPCODE &&
          execptr == (uintptr_t)&fault1_ud)
        ok |= 1;
      return;

    case 1:
      if (execptr == (uintptr_t)&fault1_ud)
        ok |= 2;
      cpu_exception_resume_pc(regs, (uintptr_t)&fault1_resume);
      return;

    case 2:
      if (type == CPU_EXCEPTION_GENERAL_PROTECTION &&
          execptr == (uintptr_t)&fault2_ud)
        ok |= 8;
      return;

    case 3:
      if (execptr == (uintptr_t)&fault2_ud)
        ok |= 16;
      cpu_exception_resume_pc(regs, (uintptr_t)&fault2_resume);
      return;

    case 4:
      if (execptr == (uintptr_t)&fault3)
        stack = stackptr;
      cpu_exception_resume_pc(regs, (uintptr_t)&fault3_resume);
      return;

#ifdef CONFIG_HEXO_USERMODE
    case 8:
      if (type == CPU_EXCEPTION_INVALID_OPCODE &&
          execptr == (uintptr_t)&syscall_fault &&
          stackptr == 1234+33)
        ok |= 512;
      cpu_exception_resume_pc(regs, (uintptr_t)&syscall_resume2);
      return;
#endif
    }
}

#ifdef CONFIG_HEXO_USERMODE

# ifdef CONFIG_CPU_X86_SYSENTER
#  define SYSCALL "sysenter\n"
# else
#  define SYSCALL "int 0x80\n"
# endif

asm(
".text \n"
"syscall:"
    "add     $17, %eax \n" // change input arg
    SYSCALL
    "add     $7, %eax \n"
    "add     $5, %eax \n"
    SYSCALL
    "add     $3, %eax \n"     // skipped
"syscall_resume:"
    "add     $8, %eax \n"
    "add     $33, %esp \n" // bad user stack
    SYSCALL
"syscall_fault:"
    "ud2    \n" // user fault
"syscall_resume2:"
    "lea    11(%eax,%edi), %eax \n"
    "mov    $0, %ebx\n" // destroy cls & tls
    "mov    %ebx, %fs\n"
    "mov    %ebx, %gs\n"
    SYSCALL
);

static CPU_SYSCALL_HANDLER(syscall_handler)
{
  printk("syscall %i\n", regs->uregs.eax);

  switch (step++)
    {
      case 5:
        if (number == 0 && regs->uregs.eax == 351+17) // syscall arg
          ok |= 64;
        return;

      case 6:
        if (number == 0 && regs->uregs.eax == 351+17+7+5) // syscall arg
          ok |= 128;
        cpu_exception_resume_pc(regs, (uintptr_t)&syscall_resume);
        return;

      case 7:
        if (number == 0 && regs->uregs.eax == 351+17+7+5+8) // syscall arg
          ok |= 256;
        regs->uregs.edi = 117;
        return;

      case 9:
        if (number == 0 && regs->uregs.eax == 117+11) // syscall arg
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
  cpu_context_set_user(1234, (uintptr_t)&syscall, 351);
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

