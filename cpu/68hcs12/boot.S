.section .boot

.extern __system_heap_end
.extern init_data_and_bss

.global cpu_boot
cpu_boot:
	/* initiate a stack */
	lds	#__system_heap_end
	/* init CCR */
	clra
	tap
	/* disable interrupts */
	sei
	/* init .data and .bss */
	bsr	init_data_and_bss
	/* jump to arch_init */
	jmp	arch_init
