SDCARDFSKK_VERSION="0.1"

EXTRA_CFLAGS += -DSDCARDFSKK_VERSION=\"$(SDCARDFSKK_VERSION)\"

obj-$(CONFIG_SDCARD_FS_KK) += sdcardfskk.o

sdcardfskk-y := dentry.o file.o inode.o main.o super.o lookup.o mmap.o packagelist.o derived_perm.o
