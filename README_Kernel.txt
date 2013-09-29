HOW TO BUILD KERNEL
1. How to Build
	- get Toolchain
		Visit http://www.codesourcery.com/, download and install Sourcery G++ Lite 2010q1 toolchain for ARM EABI.
		Extract kernel source and move into the top directory.

	- make
		$ cd kernel_src
		$ make ARCH=arm {defconfig_file}
		$ make ARCH=arm

	* defconfig_file
		- android_espresso_omap4430_r04_user_defconfig: GT-P3100/P3110/P3113

2. Output files
	- Kernel : arch/arm/boot/zImage
	- module : drivers/*/*.ko
