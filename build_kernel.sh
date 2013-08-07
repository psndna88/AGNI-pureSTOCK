#!/bin/sh
export KERNELDIR=`readlink -f .`

if [ ! -f $KERNELDIR/.config ];
then
  make defconfig psn_p31xx_oc_v2.2_defconfig
fi

. $KERNELDIR/.config

export ARCH=arm

cd $KERNELDIR/
make -j3 || exit 1

mkdir -p $KERNELDIR/BUILT/lib/modules

rm $KERNELDIR/BUILT/lib/modules/*
rm $KERNELDIR/BUILT/zImage

find -name '*.ko' -exec cp -av {} $KERNELDIR/BUILT/lib/modules/ \;
${CROSS_COMPILE}strip --strip-unneeded $KERNELDIR/BUILT/lib/modules/*
cp $KERNELDIR/arch/arm/boot/zImage $KERNELDIR/BUILT/

