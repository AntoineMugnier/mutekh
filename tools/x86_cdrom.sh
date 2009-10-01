#! /bin/sh

##
## Script made to boot from an iso cdrom file
##

if [ "x$1" = "x" ] ; then
	KERNEL=kernel-ibmpc-x86.out
else
	KERNEL=$1
fi

if [ "x$2" = "x" ] ; then
	QEMU=qemu
else
	QEMU=$2
fi

D=$(dirname $0)

TAR=tar
CP=cp
RM=rm
MKISOFS=mkisofs

ISO_NAME=mutekh.iso

$TAR xjf ${D}/iso.tar.bz2
$CP $KERNEL tmp_iso/boot/
$MKISOFS -R -b boot/grub/stage2_eltorito -no-emul-boot -boot-load-size 4 -boot-info-table -o $ISO_NAME tmp_iso/
$RM -r tmp_iso/

$QEMU -boot d -cdrom $ISO_NAME $*
