HAS_SOCLIB_CC:=$(shell (which soclib-cc 2>&1 >/dev/null && echo 1 || echo 0))
ifeq ($(HAS_SOCLIB_CC),0)
$(shell echo "SOCLIB builder \`soclib-cc' is not found, you'll experience build problems." 1>&2)
else
SOCLIB:=$(shell soclib-cc --getpath)
endif
ARCHCFLAGS=-I$(SOCLIB)/utils/include
