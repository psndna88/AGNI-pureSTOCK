#!/bin/sh
export KERNELDIR=`readlink -f .`
CROSS_COMPILE=/Working_Directory/android_prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-

export KBUILD_BUILD_USER=AGNi
export KBUILD_BUILD_HOST=PSN-pureSTOCK


if [ ! -f $KERNELDIR/.config ];
then
  make defconfig android_espresso_omap4430_r04_user_defconfig
fi

. $KERNELDIR/.config

export ARCH=arm

cd $KERNELDIR/
nice -n 10 make -j4 || exit 1

mkdir -p $KERNELDIR/BUILT/lib/modules

rm $KERNELDIR/BUILT/lib/modules/*
rm $KERNELDIR/BUILT/zImage

find -name '*.ko' -exec cp -av {} $KERNELDIR/BUILT/lib/modules/ \;
/Working_Directory/android_prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-strip --strip-unneeded $KERNELDIR/BUILT/lib/modules/*
cp $KERNELDIR/arch/arm/boot/zImage $KERNELDIR/BUILT/

