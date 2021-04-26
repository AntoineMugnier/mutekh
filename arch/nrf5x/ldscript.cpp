
#if CONFIG_NRF5X_MODEL >= 52000 && CONFIG_NRF5X_MODEL <= 52999
# define ARCH_LDSCRIPT_SECTIONS  \
	.uicr 0x10001000 : AT(0x10001000) { \
		KEEP(*(.uicr)) \
	}
#endif

#include "../common/ldscript.cpp"
