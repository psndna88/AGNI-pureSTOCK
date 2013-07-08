HOW TO BUILD KERNEL
1. How to Build
 - get Toolchain
  From Codesourcery site( http://www.codesourcery.com )
  recommand : 2010q1 version.
 - edit Makefile
   edit "CROSS_COMPILE" to right toolchain path(You downloaded).
   EX) CROSS_COMPILE ?= /opt/toolchains/arm-2010q1/bin/arm-none-linux-gnueabi-
 - make
   $ cd kernel_src
   $ make ARCH=arm {defconfig_file}
   $ make ARCH=arm
 
    * defconfig_file
      - android_espresso_omap4430_r04_user_defconfig : GT-P3100
 
2. Output files
 - Kernel : arch/arm/boot/zImage
 - module : drivers/*/*.ko