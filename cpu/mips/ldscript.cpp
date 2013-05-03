
#define __CPU_NAME_DECL(t, x) t##_##x
#define _CPU_NAME_DECL(t, x) __CPU_NAME_DECL(t, x)
#define CPU_NAME_DECL(x) _CPU_NAME_DECL(CONFIG_CPU_NAME, x)

#if defined(CONFIG_CPU_EXCEPTION_RELOCATABLE)
ASSERT((CPU_NAME_DECL(exception_vector) & 0xc0000fff) == 0x80000000, "The CPU_NAME_DECL(exception_vector) address of mips processor must match 0b10..................000000000000")
#endif

