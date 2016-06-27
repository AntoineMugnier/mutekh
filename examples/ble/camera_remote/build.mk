
all: mouse-nrf51822.bin mouse-nrf52832.bin 

flash-nrf52832: mouse-nrf52832.bin examples/ble/mouse/jlink.tmpl
	cat examples/ble/mouse/jlink.tmpl | sed -e s/DEV/nrf52832/g -e s/BIN/$</g > /tmp/jlink.tmp
	JLinkExe -if swd -CommanderScript /tmp/jlink.tmp

flash-nrf51822: mouse-nrf51822.bin examples/ble/mouse/jlink.tmpl
	cat examples/ble/mouse/jlink.tmpl | sed -e s/DEV/nrf51822/g -e s/BIN/$</g > /tmp/jlink.tmp
	JLinkExe -if swd -CommanderScript /tmp/jlink.tmp

mouse-nrf52832.bin: FORCE
	$(MAKE) CONF=examples/ble/mouse/config BUILD=nrf5x-pca10036:binary TARGET_EXT=bin

mouse-nrf51822.bin: FORCE
	$(MAKE) CONF=examples/ble/mouse/config BUILD=nrf5x-pca10028:binary TARGET_EXT=bin

.PHONY: FORCE

FORCE:
