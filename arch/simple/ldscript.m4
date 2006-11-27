
PHDRS
{
  mem PT_LOAD ;
}

SECTIONS
   {
	.text CONFIG_ARCH_SIMPLE_ROM_ADDR :
		{
			*(.boot*)
			*(.text*)
			*(.progmem*)
		} : mem

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

	__data_start = LOADADDR(.data);
	__data_end = LOADADDR(.data) + SIZEOF(.data);

	.bss CONFIG_ARCH_SIMPLE_RAM_ADDR + SIZEOF(.data) :
		{ *(.bss*) }

	__system_heap_start = .;
	__system_heap_end = CONFIG_ARCH_SIMPLE_RAM_ADDR + CONFIG_ARCH_SIMPLE_RAM_SIZE - 1;

	/* Task local data section */
 	.contextdata 0x0 :
		AT( __data_end )
		{
			/* first word contains pointer to contextdata section */
			__context_data_base = . ;
			LONG(0) ; /* fixme sizeof */

			*(.contextdata*)
		}

	__context_data_start = LOADADDR(.contextdata);
	__context_data_end = LOADADDR(.contextdata) + SIZEOF(.contextdata);
   }

ENTRY(cpu_boot)

