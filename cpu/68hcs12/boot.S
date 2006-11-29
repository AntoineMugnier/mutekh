.section .boot

.global cpu_boot
cpu_boot:
	/* initiate a stack */
	lds	#(CONFIG_ARCH_SIMPLE_RAM_ADDR + CONFIG_ARCH_SIMPLE_RAM_SIZE)
	/* init CCR */
	clra
	tap
	/* disable interrupts */
	sei
	/* jump to arch_init */
	jmp	arch_init
	/* infinite loop */
1:
	jmp	1b
