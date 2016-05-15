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

echo "Building N7105 .....";
./build_kernel_n7105.sh && sleep 10 && rm .config;
mv BUILT_*/ ../BUILT_OUTPUTS/

echo "Building N7105 CM .....";
./build_kernel_CM_n7105.sh && sleep 10 && rm .config;
mv BUILT_*/ ../BUILT_OUTPUTS/

echo "Building I605 .....";
./build_kernel_i605.sh && sleep 10 && rm .config;
mv BUILT_*/ ../BUILT_OUTPUTS/

echo "Building I605 CM .....";
./build_kernel_CM_i605.sh && sleep 10 && rm .config;
mv BUILT_*/ ../BUILT_OUTPUTS/

echo "Building I9300 .....";
./build_kernel_i9300.sh && sleep 10 && rm .config;
mv BUILT_*/ ../BUILT_OUTPUTS/

echo "Building I9300 CM .....";
./build_kernel_CM_i9300.sh && sleep 10 && rm .config;
mv BUILT_*/ ../BUILT_OUTPUTS/

echo "Building I9305 .....";
./build_kernel_i9305.sh && sleep 10 && rm .config;
mv BUILT_*/ ../BUILT_OUTPUTS/

echo "Building I9305 CM .....";
./build_kernel_CM_i9305.sh && sleep 10 && rm .config;
mv BUILT_*/ ../BUILT_OUTPUTS/

