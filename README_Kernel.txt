################################################################################
HOW TO BUILD KERNEL FOR GT-N7100_ALL

1. How to Build
	- get Toolchain
	download and install arm-2010q1 toolchain for ARM EABI.
	Extract kernel source and move into the top directory.

	$ export CROSS_COMPILE=/opt/toolchains/arm-2010q1/bin/arm-none-linux-gnueabi-
	$ make t0_04_defconfig
	$ make
	
2. Output files
	- Kernel : Kernel/arch/arm/boot/zImage
	- module : Kernel/drivers/*/*.ko
	
3. How to Clean	
    $ make clean
	
4. How to make .tar binary for downloading into target.
	- change current directory to Kernel/arch/arm/boot
	- type following command
	$ tar cvf GT-N7100_ALL.tar zImage
################################################################################