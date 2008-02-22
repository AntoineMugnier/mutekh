ifeq ($(origin HAS_SOCLIB_CC),undefined)

HAS_SOCLIB_CC:=$(shell (which soclib-cc 2>&1 >/dev/null && echo 1 || echo 0))
export HAS_SOCLIB_CC

ifeq ($(HAS_SOCLIB_CC),1)
SOCLIB:=$(shell soclib-cc --getpath)
export SOCLIB
endif

ARCHCFLAGS=-I$(SOCLIB)/utils/include
export ARCHCFLAGS

endif

ifeq ($(HAS_SOCLIB_CC),0)
$(shell echo "SOCLIB builder \`soclib-cc' is not found, you'll experience build problems." 1>&2)
endif
