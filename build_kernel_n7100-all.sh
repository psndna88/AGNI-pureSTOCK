#!/bin/sh
export KERNELDIR=`readlink -f .`

cd $KERNELDIR;

mkdir -p $KERNELDIR/../BUILT_OUTPUTS
chmod 777 $KERNELDIR/../BUILT_OUTPUTS
rm -rf $KERNELDIR/../BUILT_OUTPUTS/*
chmod 777 $KERNELDIR/build_kernel_*

echo "Building N7100 .....";
./build_kernel_n7100.sh && sleep 10 && rm .config;
mv BUILT_*/ ../BUILT_OUTPUTS/

echo "Building N7100 CM .....";
./build_kernel_CM_n7100.sh && sleep 10 && rm .config;
mv BUILT_*/ ../BUILT_OUTPUTS/


