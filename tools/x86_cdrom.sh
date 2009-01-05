#! /bin/sh

##
## Script made to boot from an iso cdrom file
##

TAR=tar
CP=cp
RM=rm
MKISOFS=mkisofs

ISO_NAME=mutekh.iso

$TAR xjf tools/iso.tar.bz2
$CP kernel-ibmpc-x86.out tmp_iso/boot/
$MKISOFS -R -b boot/grub/stage2_eltorito -no-emul-boot -boot-load-size 4 -boot-info-table -o $ISO_NAME tmp_iso/
$RM -r tmp_iso/

qemu -boot d -cdrom $ISO_NAME $*
