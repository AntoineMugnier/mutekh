CPUTOOLS=microblaze-unknown-elf-
CPUCFLAGS= -fsigned-char

# pour que les vecteurs de "reset" et "user exception" aux @0 et @8 soient écrits 
# commande "con"/"run" de XMD - Voir Chapter 9: GNU Compiler Tools Embedded System Tools Reference Manual
CPUCFLAGS += -xl-mode-novectors

ifeq ($(CONFIG_COMPILE_SOFTFLOAT), defined)
CPUCFLAGS += -msoft-float
endif

