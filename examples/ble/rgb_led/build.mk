TARGET_EXT=bin
BUILD=
export BUILD
export TARGET_EXT

all: midi-nrf51822.bin midi-nrf52832.bin 

flash-nrf52832: midi-nrf52832.bin examples/ble/midi/jlink.tmpl
	cat examples/ble/midi/jlink.tmpl | sed -e s/DEV/nrf52832/g -e s/BIN/$</g > /tmp/jlink.tmp
	JLinkExe -if swd -CommanderScript /tmp/jlink.tmp

flash-nrf51822: midi-nrf51822.bin examples/ble/midi/jlink.tmpl
	cat examples/ble/midi/jlink.tmpl | sed -e s/DEV/nrf51822/g -e s/BIN/$</g > /tmp/jlink.tmp
	JLinkExe -if swd -CommanderScript /tmp/jlink.tmp

midi-nrf52832.bin:: BUILD=nrf5x-pca10036:binary
midi-nrf51822.bin:: BUILD=nrf5x-pca10028:binary

midi-%.bin: FORCE
	@$(MAKE) CONF=examples/ble/midi/config

.PHONY: FORCE

FORCE:
