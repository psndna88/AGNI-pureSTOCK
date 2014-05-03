#!/bin/sh
export KERNELDIR=`readlink -f .`
. ~/AGNi_stamp_STOCK.sh
. ~/gcc_4.9.1_linaro_cortex-a9.sh

export ARCH=arm

if [ ! -f $KERNELDIR/.config ];
then
  make defconfig psn_i9305_v3.7_defconfig
fi

. $KERNELDIR/.config

mv .git .git-halt

cd $KERNELDIR/
make -j2 || exit 1

mkdir -p $KERNELDIR/BUILT_I9305/lib/modules

rm $KERNELDIR/BUILT_I9305/lib/modules/*
rm $KERNELDIR/BUILT_I9305/zImage

find -name '*.ko' -exec cp -av {} $KERNELDIR/BUILT_I9305/lib/modules/ \;
${CROSS_COMPILE}strip --strip-unneeded $KERNELDIR/BUILT_I9305/lib/modules/*
cp $KERNELDIR/arch/arm/boot/zImage $KERNELDIR/BUILT_I9305/

mv .git-halt .git
