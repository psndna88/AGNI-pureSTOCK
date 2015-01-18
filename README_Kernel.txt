################################################################################

1. How to Build KERNEL FOR GT-N7100
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
################################################################################

1. How to Build KERNEL FOR GT-N7105

	- get Toolchain
		From android git server , codesourcery and etc ..
		 - arm-eabi-4.4.3
		
	- edit Makefile
		edit "CROSS_COMPILE" to right toolchain path(You downloaded).
		  EX)  CROSS_COMPILE= $(android platform directory you download)/android/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-
		  Ex)  CROSS_COMPILE=/usr/local/toolchain/arm-eabi-4.4.3/bin/arm-eabi-		// check the location of toolchain
  	
        - to Build
          $ make arch=arm t0lte_04_defconfig
          $ make

2. Output files
	- Kernel : arch/arm/boot/zImage
	- module : drivers/*/*.ko

3. How to Clean	
		$ make clean
################################################################################


################################################################################

1. How to Build KERNEL FOR GT-I605
	- get Toolchain
		From android git server , codesourcery and etc ..
		 - arm-eabi-4.4.3
		
	- edit Makefile
		edit "CROSS_COMPILE" to right toolchain path(You downloaded).
		  EX)  CROSS_COMPILE= $(android platform directory you download)/android/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-
		  Ex)  CROSS_COMPILE=/usr/local/toolchain/arm-eabi-4.4.3/bin/arm-eabi-		// check the location of toolchain
  	
        - to Build
          $ make arch=arm t0vzw_04_defconfig
          $ make

2. Output files
	- Kernel : arch/arm/boot/zImage
	- module : drivers/*/*.ko

3. How to Clean	
		$ make clean
################################################################################
