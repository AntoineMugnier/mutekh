
SECTIONS
   {
	.text CONFIG_ARCH_SIMPLE_ROM_ADDR :
		{
#if (CONFIG_ARCH_SIMPLE_ROM_ADDR >= CONFIG_ARCH_SIMPLE_RESET_ADDR) && (CONFIG_ARCH_SIMPLE_ROM_ADDR + CONFIG_ARCH_SIMPLE_ROM_SIZE / 2 > CONFIG_ARCH_SIMPLE_RESET_ADDR)
# define BOOT_ADDED
			*(.boot*)
			*(.text*)
			*(.progmem*)
#elif CONFIG_ARCH_SIMPLE_ROM_ADDR + CONFIG_ARCH_SIMPLE_ROM_SIZE > CONFIG_ARCH_SIMPLE_RESET_ADDR
# define BOOT_ADDED
			*(.text*)
			*(.progmem*)
			. = CONFIG_ARCH_SIMPLE_RESET_ADDR
			*(.boot*)
#else
# define BOOT_ADDED
			*(.text*)
			*(.progmem*)
#endif
		}

#ifndef BOOT_ADDED
	.boot CONFIG_ARCH_SIMPLE_RESET_ADDR :
		{
			*(.boot*)
		}
#endif

	.data CONFIG_ARCH_SIMPLE_RAM_ADDR :
		AT ( LOADADDR(.text) + SIZEOF(.text) )
		{
			__cpu_context_data_base = . ;
			LONG(0) ; /* fixme sizeof */

			/* global variables and const data section */
			*(.data*)
			*(.rodata*)

			/* data depending on cpu architecture (fonction pointer variables, ...) */
			*(.cpuarchdata*)
		}

	__data_load_start = LOADADDR(.data);
	__data_load_end = LOADADDR(.data) + SIZEOF(.data);

	__data_start = ADDR(.data);
	__data_end = ADDR(.data) + SIZEOF(.data);

	__system_heap_start = .;
	__system_heap_end = CONFIG_ARCH_SIMPLE_RAM_ADDR + CONFIG_ARCH_SIMPLE_RAM_SIZE - 1;

	/* Task local data section */
 	.contextdata 0x0 :
		AT( __data_load_end )
		{
			/* first word contains pointer to contextdata section */
			__context_data_base = . ;
			LONG(0) ; /* fixme sizeof */

			*(.contextdata*)
		}

	__context_data_start = LOADADDR(.contextdata);
	__context_data_end = LOADADDR(.contextdata) + SIZEOF(.contextdata);

	.bss __data_end :
		{ *(.bss*) }

	__bss_start = ADDR(.bss);
	__bss_end = ADDR(.bss) + SIZEOF(.bss);

   }

ENTRY(cpu_boot)

