#!/bin/sh
export KERNELDIR=`readlink -f .`
. ~/AGNi_stamp_CM.sh
. ~/gcc-linaro-5.3-2016.02_arm-gnueabi.sh

export ARCH=arm

if [ ! -f $KERNELDIR/.config ];
then
  make defconfig psn_i605_new_cm_defconfig
fi

. $KERNELDIR/.config

mv .git .git-halt

cd $KERNELDIR/
make -j3 || exit 1

mkdir -p $KERNELDIR/BUILT_I605_CM/lib/modules

rm $KERNELDIR/BUILT_I605_CM/lib/modules/*
rm $KERNELDIR/BUILT_I605_CM/zImage

find -name '*.ko' -exec cp -av {} $KERNELDIR/BUILT_I605_CM/lib/modules/ \;
${CROSS_COMPILE}strip --strip-unneeded $KERNELDIR/BUILT_I605_CM/lib/modules/*
cp $KERNELDIR/arch/arm/boot/zImage $KERNELDIR/BUILT_I605_CM/

mv .git-halt .git
