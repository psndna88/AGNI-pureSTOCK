/*
 * fs/sdcardfskk/mmap.c
 *
 * Copyright (c) 2013 Samsung Electronics Co. Ltd
 *   Authors: Daeho Jeong, Woojoong Lee, Seunghwan Hyun,
 *               Sunghwan Yun, Sungjong Seo
 *
 * This program has been developed as a stackable file system based on
 * the WrapFS which written by
 *
 * Copyright (c) 1998-2011 Erez Zadok
 * Copyright (c) 2009     Shrikar Archak
 * Copyright (c) 2003-2011 Stony Brook University
 * Copyright (c) 2003-2011 The Research Foundation of SUNY
 *
 * This file is dual licensed.  It may be redistributed and/or modified
 * under the terms of the Apache 2.0 License OR version 2 of the GNU
 * General Public License.
 */

#include "sdcardfskk.h"

static int sdcardfskk_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	int err;
	struct file *file, *lower_file;
	const struct vm_operations_struct *lower_vm_ops;
	struct vm_area_struct lower_vma;

	memcpy(&lower_vma, vma, sizeof(struct vm_area_struct));
	file = lower_vma.vm_file;
	lower_vm_ops = SDCARDFSKK_F(file)->lower_vm_ops;
	BUG_ON(!lower_vm_ops);

	lower_file = sdcardfskk_lower_file(file);
	/*
	 * XXX: vm_ops->fault may be called in parallel.  Because we have to
	 * resort to temporarily changing the vma->vm_file to point to the
	 * lower file, a concurrent invocation of sdcardfskk_fault could see a
	 * different value.  In this workaround, we keep a different copy of
	 * the vma structure in our stack, so we never expose a different
	 * value of the vma->vm_file called to us, even temporarily.  A
	 * better fix would be to change the calling semantics of ->fault to
	 * take an explicit file pointer.
	 */
	lower_vma.vm_file = lower_file;
	err = lower_vm_ops->fault(&lower_vma, vmf);
	return err;
}

static ssize_t sdcardfskk_direct_IO(int rw, struct kiocb *iocb,
			      const struct iovec *iov, loff_t offset,
			      unsigned long nr_segs)
{
	/*
     * This function returns zero on purpose in order to support direct IO.
	 * __dentry_open checks a_ops->direct_IO and returns EINVAL if it is null.
     *
	 * However, this function won't be called by certain file operations
     * including generic fs functions.  * reads and writes are delivered to
     * the lower file systems and the direct IOs will be handled by them.
	 *
     * NOTE: exceptionally, on the recent kernels (since Linux 3.8.x),
     * swap_writepage invokes this function directly.
	 */
	printk(KERN_INFO "%s, operation is not supported\n", __func__);
	return 0;
}

/*
 * XXX: the default address_space_ops for sdcardfskk is empty.  We cannot set
 * our inode->i_mapping->a_ops to NULL because too many code paths expect
 * the a_ops vector to be non-NULL.
 */
const struct address_space_operations sdcardfskk_aops = {
	/* empty on purpose */
	.direct_IO	= sdcardfskk_direct_IO,
};

const struct vm_operations_struct sdcardfskk_vm_ops = {
	.fault		= sdcardfskk_fault,
};
