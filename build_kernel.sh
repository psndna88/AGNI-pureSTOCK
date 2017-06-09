#!/bin/sh
export KERNELDIR=`readlink -f .`
. ~/WORKING_DIRECTORY/AGNi_stamp_STOCK.sh
#. ~/WORKING_DIRECTORY/gcc-4.9-uber_arm-eabi.sh
. ~/WORKING_DIRECTORY/gcc-6.x-uber_arm-eabi.sh

echo ""
echo " Cross-compiling AGNi P3100 pureSTOCK-KK-4.1.2 kernel ..."
echo ""

cd $KERNELDIR/

if [ ! -f $KERNELDIR/.config ];
then
    make agni_p3100_defconfig
fi

mv .git .git-halt
make -j3 || exit 1
mv .git-halt .git


rm -rf $KERNELDIR/BUILT-P3100
mkdir -p $KERNELDIR/BUILT-P3100/lib/modules

echo ""
echo "BEGINING SGX540 PVR KM COMPILATION ..........."
cd $KERNELDIR/pvr_source/eurasiacon/build/linux2/omap4430_android
make clean
make TARGET_PRODUCT="blaze_tablet" BUILD=release TARGET_SGX=540 PLATFORM_VERSION=4.1.2 || exit
mv $KERNELDIR/pvr_source/eurasiacon/binary2_540_120_omap4430_android_release/target/*.ko $KERNELDIR/BUILT-P3100/lib/modules/
make clean
rm -rf $KERNELDIR/pvr_source/eurasiacon/binary2_540_120_omap4430_android_release

mv $KERNELDIR/arch/arm/boot/zImage $KERNELDIR/BUILT-P3100/
cd $KERNELDIR; find -name '*.ko' -exec mv -v {} $KERNELDIR/BUILT-P3100/lib/modules/ \;


echo ""
echo "AGNi pureSTOCK-KK-4.1.2 has been built for P3100 !!!"

