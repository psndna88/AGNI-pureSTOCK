################################################################################
HOW TO BUILD KERNEL FOR GT-N7100_SWA

1. How to Build
	- get Toolchain
	download and install arm-eabi-4.4.3 toolchain for ARM EABI.
	Extract kernel source and move into the top directory.

	$ export CROSS_COMPILE=/opt/toolchains/arm-eabi-4.4.3/bin/arm-eabi-
	$ make t0_04_defconfig
	  OR
	$ make arch=arm t0lte_04_defconfig
	  OR
	$ make arch=arm t0vzw_04_defconfig
	$ make
	
2. Output files
	- Kernel : Kernel/arch/arm/boot/zImage
	- module : Kernel/drivers/*/*.ko
	
3. How to Clean	
    $ make clean
################################################################################