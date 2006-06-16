#!/bin/sh

if test 2 -ne $# ; then
    echo "setup-links.sh [arch name] [cpu name]"
    exit
fi

if ! test -d arch/$1 ; then
    echo "unsupported arch $1"
    exit
fi

if ! test -d arch/$1 ; then
    echo "unsupported arch $1"
    exit
fi

rm -f arch/current
ln -sf $1 arch/current
rm -f cpu/current
ln -sf $2 cpu/current

