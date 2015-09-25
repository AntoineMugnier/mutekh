TARGET_SECTIONS+= .pic32fuses
OBJCOPYFLAGS = --change-section-lma '.pic32fuses-0xffffffff80000000' --change-section-lma '.text-0xffffffff80000000' --change-section-lma '.rodata-0xffffffff80000000' --change-section-lma '.data-0xffffffff80000000' --change-section-lma '.contextdata-0xffffffff80000000' --change-section-lma '.reset-0xffffffffa0000000'
LD_NO_Q = 1

