


/** Magic value must be stored here to enable other registers. 0 must
    be stored to exit magix state. */
#define SOCLIB_MC_MAGIC (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 0)

/** Data register 1, depends on action performed */
#define SOCLIB_MC_R1 (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 4)

/** Data register 1, depends on action performed */
#define SOCLIB_MC_R2 (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 8)

/** Creates a new context: R1 = base, R2 = size, value = id */
#define SOCLIB_MC_CTX_CREATE (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 12)

/** Delete a context: value = id */
#define SOCLIB_MC_CTX_DELETE (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 16)

/** Change a context id: R1 = old_id, value = new_id */
#define SOCLIB_MC_CTX_CHANGE (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 20)

/** Set current context id */
#define SOCLIB_MC_CTX_SET (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 24)

/** Set allocated region state: R1 = base, R2 = size, value = state */
#define SOCLIB_MC_REGION_UPDATE (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 28)
/** Allocatable region state is free */
# define SOCLIB_MC_REGION_FREE 1
/** Allocatable region state is allocated */
# define SOCLIB_MC_REGION_ALLOC 2

/** Set enabled checks */
#define SOCLIB_MC_ENABLE (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 32)

/** Stack pointer range must be checked */
# define SOCLIB_MC_CHECK_SP 1
/** Memory initialization status must be checked */
# define SOCLIB_MC_CHECK_INIT 2
/** Memory allocation status must be checked */
# define SOCLIB_MC_CHECK_REGIONS 4
/** Frame pointer range must be checked */
# define SOCLIB_MC_CHECK_FP 8

#define SOCLIB_MC_MAGIC_VAL 0x4d656d63

#define ASM_STR_(x) #x
#define ASM_STR(x) ASM_STR_(x)

