
#define TYPES_H_
#include <cpu/hexo/types.h>

#define __CPU_NAME_DECL(t, x) t##_##x
#define _CPU_NAME_DECL(t, x) __CPU_NAME_DECL(t, x)
#define CPU_NAME_DECL(x) _CPU_NAME_DECL(CONFIG_CPU_NAME, x)

#define LMAEND(section) (LOADADDR(section) + SIZEOF(section))
#define VMAEND(section) (ADDR(section) + SIZEOF(section))

#if defined(CONFIG_LOAD_BOOTLOAD)
# define RO_START CONFIG_LOAD_BOOTLOAD_KERNEL_ADDR
#elif defined(CONFIG_LOAD_ROM)
# define RO_START CONFIG_LOAD_ROM_RO_ADDR
#else
# error The generic linker script requires either CONFIG_LOAD_BOOTLOAD or CONFIG_LOAD_ROM
#endif

SECTIONS
{

#ifdef ARCH_LDSCRIPT_SECTIONS
  ARCH_LDSCRIPT_SECTIONS
#endif

#ifdef CONFIG_LOAD_MULTIBOOT
        .multiboot RO_START : AT(RO_START) {
# undef RO_START
# define RO_START LMAEND(.multiboot)
                LONG(0x1badb002);     /* magic */
# ifdef CONFIG_LOAD_MULTIBOOT_ELF
                LONG(0);             /* flags */
                LONG(-0x1badb002);   /* checksum */
# else
                LONG(0x00010000);             /* flags */
                LONG(-(0x1badb002+0x00010000)); /* checksum */
		LONG(LOADADDR(.multiboot));     /* header_addr */
		LONG(LOADADDR(.multiboot));     /* load addr */
		LONG(__cpu_data_end);           /* load addr end */
		LONG(__bss_end);                /* bss addr end */
		LONG(mutekh_entry);             /* entry point */
# endif
        }
#endif

/**************************************** reset vectors */

#ifdef CONFIG_LOAD_RESET_SEPARATE
	.reset CONFIG_CPU_RESET_ADDR : AT(CONFIG_CPU_RESET_ADDR) {
		KEEP(*(.reset))
	}

        ASSERT(SIZEOF(.reset) != 0, "Empty .reset section")
#endif

#ifdef CONFIG_LOAD_SMP_RESET_SEPARATE
	.smpreset CONFIG_CPU_SMP_RESET_ADDR : AT(CONFIG_CPU_SMP_RESET_ADDR) {
		KEEP(*(.smpreset))
	}

        ASSERT(SIZEOF(.smpreset) != 0, "Empty .smpreset section")
#endif

#ifdef CONFIG_LOAD_SMP_RESET_COPY
	.smpreset CONFIG_CPU_SMP_RESET_ADDR : AT(RO_START) {
		KEEP(*(.smpreset))
                CPU_NAME_DECL(smp_reset_vector_end) = .;
	}

        ASSERT(SIZEOF(.smpreset) != 0, "Empty .smpreset section")

	__smp_reset_load_start = LOADADDR(.smpreset);
	__smp_reset_load_end = LMAEND(.smpreset);

# define EXCEP_START LMAEND(.smpreset)
#else
# define EXCEP_START RO_START
#endif

#ifdef CONFIG_LOAD_SMP_RESET_MERGED
        CPU_NAME_DECL(smp_reset_vector) = CPU_NAME_DECL(reset_vector);
#endif

/**************************************** exception vectors */

#ifdef CONFIG_LOAD_EXCEPTIONS_SEPARATE
	.excep CONFIG_LOAD_EXCEPTIONS_ADDR : AT(CONFIG_LOAD_EXCEPTIONS_ADDR) {
		KEEP(*(.excep))
                CPU_NAME_DECL(exception_vector_end) = .;
	}

#endif

#ifdef CONFIG_LOAD_EXCEPTIONS_COPY

	.excep CONFIG_LOAD_EXCEPTIONS_ADDR : AT(EXCEP_START) {
		KEEP(*(.excep))
                CPU_NAME_DECL(exception_vector_end) = .;
	}


	__exception_load_start = LOADADDR(.excep);
	__exception_load_end = LMAEND(.excep);

# define TEXT_START LMAEND(.excep)
#else
# define TEXT_START EXCEP_START
#endif

/**************************************** code */

        .text TEXT_START : AT(TEXT_START) {

#ifdef CONFIG_LOAD_EXCEPTIONS_TEXT
                . = ALIGN(CONFIG_CPU_EXCEPTION_ALIGN);
                CPU_NAME_DECL(exception_vector_start) = .;
		KEEP(*(.excep))
                CPU_NAME_DECL(exception_vector_end) = .;

#endif

#ifdef CONFIG_LOAD_SMP_RESET_TEXT
                . = ALIGN(CONFIG_CPU_SMP_RESET_ALIGN);
                CPU_NAME_DECL(smp_reset_vector_start) = .;
		KEEP(*(.smpreset))
                CPU_NAME_DECL(smp_reset_vector_end) = .;

                ASSERT(CPU_NAME_DECL(smp_reset_vector_start) != CPU_NAME_DECL(smp_reset_vector_end),
                       "Empty .smpreset section");
#endif
		*(.init*)
		*(.text*)
		*(.glue*)
		*(.got2)
	}

/**************************************** rodata */

	.rodata VMAEND(.text) : AT(LMAEND(.text)) {
                . = ALIGN(16);
		*(.rodata*)

                . = ALIGN(8);
#ifdef CONFIG_DEVICE_DRIVER_REGISTRY
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
	}

/**************************************** data */

#if defined(CONFIG_LOAD_ROM)
	.data CONFIG_LOAD_ROM_RW_ADDR : AT(LMAEND(.rodata)) {
#else
	.data VMAEND(.rodata) : AT(LMAEND(.rodata)) {
#endif
		. = ALIGN(16);
		__data_start = ABSOLUTE(.);
		*(.sdata*)
		*(.data*)
		*(.cpuarchdata*)

#if defined(CONFIG_DEVICE)
                . = ALIGN(8);
		dev_devices_table = .;
		KEEP(*(.devices .devices.*))
		dev_devices_table_end = .;
#endif
#ifndef CONFIG_ARCH_SMP
                *(.cpudata*) 
#endif
        }

	__data_load_start = LOADADDR(.data);
	__data_load_end = LMAEND(.data);

	/* Context local data template section */
	.contextdata 0x0 : AT(__data_load_end) {
		. = ALIGN(16);
                *(.contextdata*) 
	}

	__context_data_start = LOADADDR(.contextdata);
	__context_data_end = LMAEND(.contextdata);

#ifdef CONFIG_ARCH_SMP
	/* CPU local data template section */
        .cpudata 0x0 : AT(__context_data_end) {
		. = ALIGN(16);
                *(.cpudata*) 
        }

	__cpu_data_start = LOADADDR(.cpudata);
	__cpu_data_end = LMAEND(.cpudata);
#else
# define __cpu_data_end __context_data_end
#endif

        /*
          End of all ROM sections (excluding separate reset, if any).
          This is start address in flash that can be used for other
          purposes in a MCU.
        */
        __rom_end = __cpu_data_end;


#if defined(CONFIG_LOAD_ROM)
	.bss VMAEND(.data) : AT(VMAEND(.data)) {
#else
	.bss __cpu_data_end : AT(__cpu_data_end) {
#endif
		. = ALIGN(16);
		__bss_start = ABSOLUTE(.);
		*(.sbss*)
		*(COMMON)
		*(.common*)
		*(.scommon*)
		*(.bss*)
		__bss_end = ABSOLUTE(.);
	}

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
          /* DWARF 5+ */
          .debug_rnglists     0 : { *(.debug_rnglists) }
          .debug_line_str     0 : { *(.debug_line_str) }

 	/DISCARD/ : { *(.*) }

}

#ifndef CONFIG_CPU_EXCEPTION_RELOCATABLE
ASSERT(CPU_NAME_DECL(exception_vector) == CONFIG_CPU_EXCEPTION_ADDR,
       "The address of the [CPU_NAME]_exception_vector symbol is not equal to CONFIG_CPU_EXCEPTION_ADDR and the processor doesn't support relocating the exception base address.")
#endif

ASSERT(CPU_NAME_DECL(exception_vector) % CONFIG_CPU_EXCEPTION_ALIGN == 0,
       "The address of the [CPU_NAME]_exception_vector symbol is not aligned on CONFIG_CPU_EXCEPTION_ALIGN.")

#ifdef CONFIG_LOAD_ROM

#if CONFIG_CPU_RESET_SIZE != 0
ASSERT(CPU_NAME_DECL(reset_vector) == CONFIG_CPU_RESET_ADDR,
        "The [CPU_NAME]_reset_vector symbol is not equal to CONFIG_CPU_RESET_ADDR.")
#endif

ASSERT(LMAEND(.rodata) <= CONFIG_LOAD_ROM_RO_ADDR + CONFIG_LOAD_ROM_RO_SIZE,
       ".text + .rodata do not fit in CONFIG_LOAD_ROM_RO_SIZE bytes");

ASSERT(LMAEND(.bss) <= CONFIG_LOAD_ROM_RW_ADDR + CONFIG_LOAD_ROM_RW_SIZE,
       ".data + .bss do not fit in CONFIG_LOAD_ROM_RW_SIZE bytes");

# if defined(CONFIG_CPU_RESET_SEPARATE) && !defined(CONFIG_LOAD_RESET_SEPARATE)
#  error CONFIG_LOAD_ROM: The selected processor requires a separate section for the reset vector
# endif

# if !defined(CONFIG_LOAD_RESET_SEPARATE) && !defined(CONFIG_LOAD_EXCEPTIONS_SEPARATE) && CONFIG_CPU_RESET_SIZE != 0
#  warning CONFIG_LOAD_ROM: No separate .reset or .except sections to hold the reset vector
# endif
#endif

#ifdef CONFIG_ARCH_SMP
ASSERT(CPU_NAME_DECL(smp_reset_vector) % CONFIG_CPU_SMP_RESET_ALIGN == 0,
       "The address of the [CPU_NAME]_smp_reset_vector symbol is not aligned on CONFIG_CPU_SMP_RESET_ALIGN.")

ASSERT(CPU_NAME_DECL(smp_reset_vector) == CONFIG_CPU_SMP_RESET_ADDR,
       "The address of the [CPU_NAME]_smp_reset_vector symbol is not equal to CONFIG_CPU_SMP_RESET_ADDR.")
#endif

ENTRY(mutekh_entry)

