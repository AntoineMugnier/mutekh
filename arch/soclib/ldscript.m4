
m4_include(soclib_addresses.ldscript.m4)

/*
PHDRS
{
  boot PT_LOAD ;
  excep PT_LOAD ;
  mem PT_LOAD ;
}
*/

SECTIONS
   {
	. = dsx_segment_reset_start;
	.boot : { *(.boot*) }/* : boot*/

	. = dsx_segment_excep_start;
	.excep : { *(.excep*) }/* : excep*/

	. = dsx_segment_text_start;
	.text : { *(.text*) }/* : mem*/

	. = dsx_segment_data_uncached_start;

	. = ALIGN(16);
	_gp = . + 0x7ff0;	/* global data register pointer */

	.sdata : { *(.sdata*) }
	.sbss :	 { *(.sbss*) }

	.bss :	 { *(.bss*) }

	.data :
		{
			/* global variables and const data section */
			*(.data*)
			*(.rodata*)
			global_driver_registry = .;
			*(.drivers)
			global_driver_registry_end = .;

			/* data depending on cpu architecture (fonction pointer variables, ...) */
			*(.cpuarchdata*)
		}

	/* CPU local data section */
	.cpudata  0x0 : AT( LOADADDR(.data) + SIZEOF(.data) )
		{
			*(.cpudata*)
		}

	__cpu_data_start = LOADADDR(.cpudata);
	__cpu_data_end = LOADADDR(.cpudata) + SIZEOF(.cpudata);

	/* Task local data section */
 	.contextdata 0x0 : AT( __cpu_data_end )
		{
			*(.contextdata*)
		}

	__segment_excep_start =  LOADADDR(.excep);
	__segment_excep_end =  LOADADDR(.excep) + SIZEOF(.excep);

	__segment_text_start =  LOADADDR(.text);
	__segment_text_end =  LOADADDR(.text) + SIZEOF(.text);

	__segment_data_uncached_start = dsx_segment_data_uncached_start;

	__context_data_start = LOADADDR(.contextdata);
	__context_data_end = LOADADDR(.contextdata) + SIZEOF(.contextdata);

	/* GOT section */
 	.got2 :
		{
			*(.got2)
		}

	__system_uncached_heap_start = __context_data_end;
	__system_uncached_heap_end = dsx_segment_data_uncached_end;

	__system_cached_heap_start = dsx_segment_data_cached_start;
	__system_cached_heap_end = dsx_segment_data_cached_end;
   }

ENTRY(cpu_boot)

