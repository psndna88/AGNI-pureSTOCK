/*
 * The sdcardfslp
 *
 * Copyright (c) 2013 Samsung Electronics Co. Ltd
 *   Authors: Daeho Jeong, Woojoong Lee, Kitae Lee, Yeongjin Gil
 *
 * Revision History
 * 2014.06.24 : Release Version 2.1.0
 *    - Add sdcardfslp version
 *    - Add kernel log when put_super
 * 2014.07.21 : Release Version 2.1.1
 *    - Add sdcardfslp_copy_inode_attr() to fix permission issue
 *    - Delete mmap_sem lock in sdcardfslp_setattr() to avoid deadlock
 */

#define SDCARDFSLP_VERSION "2.1.1"
