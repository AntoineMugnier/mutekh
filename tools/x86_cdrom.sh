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
	QEMU=$(type -p qemu 2>/dev/null)
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
$CP $KERNEL tmp_iso/boot/kernel-ibmpc-x86.out
$MKISOFS -R -b boot/grub/stage2_eltorito -no-emul-boot -boot-load-size 4 -boot-info-table -o $ISO_NAME tmp_iso/
$RM -r tmp_iso/

if [ ! -z "$QEMU" -a -e "$QEMU" ] ; then
	$QEMU -boot d -cdrom $ISO_NAME $*
fi
