
PHDRS
{
  mem PT_LOAD ;
}

SECTIONS
   {
	.text 0x40100000 : AT( 0x40100000 ) { *(.text*) } : mem

        .data : ALIGN(16)
		{
			/* global variables and const data section */
                        . = ALIGN(16);
			*(.data*)
                        . = ALIGN(16);
			*(.rodata*)

                        . = ALIGN(8);
#if defined(CONFIG_DEVICE)
                        dev_devices_table = .;
                        KEEP(*(.devices .devices.*))
                        dev_devices_table_end = .;

			driver_registry_table = .;
			KEEP(*(.drivers .drivers.*))
			driver_registry_table_end = .;
#endif
#ifdef CONFIG_MUTEK_SHELL
                        shell_cmd_table = .;
	                KEEP(*(.shell .shell.*))
                        LONG(0);        /* NULL terminated array */
# if INT_PTR_SIZE == 64
                        LONG(0);
# endif
#endif

			/* data depending on cpu architecture (fonction pointer variables, ...) */
			*(.cpuarchdata*)
		}
        __data_start = LOADADDR(.data);
    __data_end = LOADADDR(.data) + SIZEOF(.data);

	.bss : { *(.bss*) }
    __bss_start = LOADADDR(.bss);
    __bss_end = LOADADDR(.bss) + SIZEOF(.bss);

	/* We do not set VMA to 0 for these sections because it causes some loading
	problems with linux exec loader (when /proc/sys/vm/mmap_min_addr is set).
	Instead we subtract the __context_data_start to the TLS address when used. */

    /* ensure cpudata is in a separate page from data */
    . = ALIGN(CONFIG_ARCH_EMU_PAGESIZE);

	/* CPU local data section */
	.cpudata :
		{
			*(.cpudata*)
		}

	__cpu_data_start = LOADADDR(.cpudata);
	__cpu_data_end = LOADADDR(.cpudata) + SIZEOF(.cpudata);

	/* Task local data section */
 	.contextdata :
		{
			*(.contextdata*)
		}

	__context_data_start = LOADADDR(.contextdata);
	__context_data_end = LOADADDR(.contextdata) + SIZEOF(.contextdata);

	__system_heap_start = __context_data_end;

#warning bad __initial_stack
__initial_stack = __data_end;

          /* DWARF 1 */
          .debug          0 : { *(.debug) }
          .line           0 : { *(.line) }
          /* GNU DWARF 1 extensions */
          .debug_srcinfo  0 : { *(.debug_srcinfo) }
          .debug_sfnames  0 : { *(.debug_sfnames) }
          /* DWARF 1.1 and DWARF 2 */
          .debug_ranges  0 : { *(.debug_ranges) }
          .debug_aranges  0 : { *(.debug_aranges) }
          .debug_pubnames 0 : { *(.debug_pubnames) }
          .debug_pubtypes 0 : { *(.debug_pubtypes) }
          /* DWARF 2 */
          .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
          .debug_abbrev   0 : { *(.debug_abbrev) }
          .debug_line     0 : { *(.debug_line) }
          .debug_frame    0 : { *(.debug_frame) }
          .debug_str      0 : { *(.debug_str) }
          .debug_loc      0 : { *(.debug_loc) }
          .debug_macinfo  0 : { *(.debug_macinfo) }
          /* SGI/MIPS DWARF 2 extensions */
          .debug_weaknames 0 : { *(.debug_weaknames) }
          .debug_funcnames 0 : { *(.debug_funcnames) }
          .debug_typenames 0 : { *(.debug_typenames) }
          .debug_varnames  0 : { *(.debug_varnames) }
   }

ENTRY(mutekh_startup)

