
#ifdef CONFIG_ARCH_SMP
# ifdef CONFIG_CPU_SPARC
#  include "cpu/sparc/include/cpu/hexo/lock_ldstub.h"
# else
#  error No lock implementation defined for this arch/cpu pair
# endif
#else
# include "arch/common/include/arch/hexo/lock_na.h"
#endif

