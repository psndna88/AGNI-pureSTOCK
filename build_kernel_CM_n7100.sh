#!/bin/sh
export KERNELDIR=`readlink -f .`
. ~/AGNi_stamp_CM.sh
. ~/gcc_4.9.3_linaro_cortex-a9.sh

export ARCH=arm

if [ ! -f $KERNELDIR/.config ];
then
  make defconfig psn_n7100_new_cm_defconfig
fi

. $KERNELDIR/.config

mv .git .git-halt

cd $KERNELDIR/
make -j3 || exit 1

mkdir -p $KERNELDIR/BUILT_N7100_CM/lib/modules

rm $KERNELDIR/BUILT_N7100_CM/lib/modules/*
rm $KERNELDIR/BUILT_N7100_CM/zImage

find -name '*.ko' -exec cp -av {} $KERNELDIR/BUILT_N7100_CM/lib/modules/ \;
${CROSS_COMPILE}strip --strip-unneeded $KERNELDIR/BUILT_N7100_CM/lib/modules/*
cp $KERNELDIR/arch/arm/boot/zImage $KERNELDIR/BUILT_N7100_CM/

mv .git-halt .git
