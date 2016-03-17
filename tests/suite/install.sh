#!/bin/bash

DESTDIR=$PWD

echo
echo "########################################################################"
echo "#                   Test environmant installer                         #"
echo "########################################################################"
echo

mkdir -p $DESTDIR/build/usr/{bin,lib}
mkdir -p $DESTDIR/downloads

echo "#### Install toolchains ####"
echo

GCC_VERSION="4.9.3"
GCC_TARGETS="arm i686 lm32 m68k microblaze mipsel nios2 powerpc sparc x86_64"
GCC_MUTEKH_URL_BASE="https://www.mutekh.org/tools/precompiled"

if [ ! -d $DESTDIR/toolchains ]; then
    mkdir -p $DESTDIR/toolchains/
fi

pushd $PWD/toolchains 2>&1 > /dev/null

for target in $GCC_TARGETS; do
    url="$GCC_MUTEKH_URL_BASE/mutekh_${target}_toolchain-$GCC_VERSION-i386-linux.tar.gz"

    echo ">>> Downloading toolchain for target $target..."
    curl -L -k -s $url > $DESTDIR/downloads/`basename $url`

    if [ $? -eq 0 ]; then
        echo ">>> Extracting toolchain for target $target..."
        tar xzf $DESTDIR/downloads/`basename $url` -C $DESTDIR/build
        for bin in $DESTDIR/build/opt/mutekh/bin/*; do
            ln -sf $bin $DESTDIR/build/usr/bin/`basename $bin`
        done
    else
        echo ">>> Failed to download toolchain!"
    fi
done

popd 2>&1 > /dev/null

echo "#### Install SystemCASS simulator ####"
echo

if [ ! -f $DESTDIR/usr/lib/libsystemc.so ]; then
    pushd $DESTDIR/downloads 2>&1 > /dev/null
    svn co https://www.soclib.fr/svn/systemcass/sources systemcass
    (cd systemcass; \
        ./bootstrap; \
        mkdir build; \
        cd build; \
        ../configure --prefix=/usr; \
        make DESTDIR=$DESTDIR/build install \
    )
    popd 2>&1 > /dev/null
fi

echo "#### Install/update SoCLib SDK ####"
echo

if [ ! -d $DESTDIR/soclib ]; then
    hg clone https://www.soclib.org/hg/soclib-ng soclib
else
    pushd $DESTDIR/soclib 2>&1 > /dev/null
    hg pull -u
    popd 2>&1 > /dev/null
fi
