#!/bin/sh
export KERNELDIR=`readlink -f .`
. ~/AGNi_stamp_STOCK.sh
. ~/gcc_4.4.3.sh

mv .git .git-halt

export ARCH=arm

if [ ! -f $KERNELDIR/.config ];
then
  make defconfig psn_p31xx_oc_v2.2_defconfig
fi

. $KERNELDIR/.config

cd $KERNELDIR/
make -j3 || exit 1

mkdir -p $KERNELDIR/BUILT/lib/modules

rm $KERNELDIR/BUILT/lib/modules/*
rm $KERNELDIR/BUILT/zImage

find -name '*.ko' -exec cp -av {} $KERNELDIR/BUILT/lib/modules/ \;
${CROSS_COMPILE}strip --strip-unneeded $KERNELDIR/BUILT/lib/modules/*
cp $KERNELDIR/arch/arm/boot/zImage $KERNELDIR/BUILT/

mv .git-halt .git
