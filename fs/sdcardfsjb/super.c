/*
 * Copyright (c) 1998-2011 Erez Zadok
 * Copyright (c) 2009	   Shrikar Archak
 * Copyright (c) 2003-2011 Stony Brook University
 * Copyright (c) 2003-2011 The Research Foundation of SUNY
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "sdcardfsjb.h"

/*
 * The inode cache is used with alloc_inode for both our inode info and the
 * vfs inode.
 */
static struct kmem_cache *sdcardfsjb_inode_cachep;

/* final actions when unmounting a file system */
static void sdcardfsjb_put_super(struct super_block *sb)
{
	struct sdcardfsjb_sb_info *spd;
	struct super_block *s;

	spd = SDCARDFSJB_SB(sb);
	if (!spd)
		return;

	/* decrement lower super references */
	s = sdcardfsjb_lower_super(sb);
	sdcardfsjb_set_lower_super(sb, NULL);
	atomic_dec(&s->s_active);

	kfree(spd);
	sb->s_fs_info = NULL;
}

static int sdcardfsjb_statfs(struct dentry *dentry, struct kstatfs *buf)
{
	int err;
	struct path lower_path;

#if defined(LOWER_FS_MIN_FREE_SIZE)
	u32 min_blocks;
#endif

	sdcardfsjb_get_lower_path(dentry, &lower_path);
	err = vfs_statfs(&lower_path, buf);
	sdcardfsjb_put_lower_path(dentry, &lower_path);

#if defined(LOWER_FS_MIN_FREE_SIZE)
	/* Invalid statfs informations. */
	if (buf->f_bsize == 0) {
		printk(KERN_ERR "Returned block size is zero.\n");
		return -EINVAL;
	}

	min_blocks = (LOWER_FS_MIN_FREE_SIZE/buf->f_bsize);
	buf->f_blocks -= min_blocks;

	if (buf->f_bavail > min_blocks)
		buf->f_bavail -= min_blocks;
	else
		buf->f_bavail = 0;

	/* Make reserved blocks invisiable to media storage */
	buf->f_bfree = buf->f_bavail;
#endif
	/* set return buf to our f/s to avoid confusing user-level utils */
	buf->f_type = SDCARDFS_SUPER_MAGIC;

	return err;
}

/*
 * @flags: numeric mount options
 * @options: mount options string
 */
static int sdcardfsjb_remount_fs(struct super_block *sb, int *flags, char *options)
{
	int err = 0;

	/*
	 * The VFS will take care of "ro" and "rw" flags among others.  We
	 * can safely accept a few flags (RDONLY, MANDLOCK), and honor
	 * SILENT, but anything else left over is an error.
	 */
	if ((*flags & ~(MS_RDONLY | MS_MANDLOCK | MS_SILENT)) != 0) {
		printk(KERN_ERR
		       "sdcardfsjb: remount flags 0x%x unsupported\n", *flags);
		err = -EINVAL;
	}

	return err;
}

/*
 * Called by iput() when the inode reference count reached zero
 * and the inode is not hashed anywhere.  Used to clear anything
 * that needs to be, before the inode is completely destroyed and put
 * on the inode free list.
 */
static void sdcardfsjb_evict_inode(struct inode *inode)
{
	struct inode *lower_inode;

	truncate_inode_pages(&inode->i_data, 0);
	end_writeback(inode);
	/*
	 * Decrement a reference to a lower_inode, which was incremented
	 * by our read_inode when it was created initially.
	 */
	lower_inode = sdcardfsjb_lower_inode(inode);
	sdcardfsjb_set_lower_inode(inode, NULL);
	iput(lower_inode);
}

static struct inode *sdcardfsjb_alloc_inode(struct super_block *sb)
{
	struct sdcardfsjb_inode_info *i;

	i = kmem_cache_alloc(sdcardfsjb_inode_cachep, GFP_KERNEL);
	if (!i)
		return NULL;

	/* memset everything up to the inode to 0 */
	memset(i, 0, offsetof(struct sdcardfsjb_inode_info, vfs_inode));

	i->vfs_inode.i_version = 1;
	return &i->vfs_inode;
}

static void sdcardfsjb_destroy_inode(struct inode *inode)
{
	kmem_cache_free(sdcardfsjb_inode_cachep, SDCARDFSJB_I(inode));
}

/* sdcardfsjb inode cache constructor */
static void init_once(void *obj)
{
	struct sdcardfsjb_inode_info *i = obj;

	inode_init_once(&i->vfs_inode);
}

int sdcardfsjb_init_inode_cache(void)
{
	int err = 0;

	sdcardfsjb_inode_cachep =
		kmem_cache_create("sdcardfsjb_inode_cache",
				  sizeof(struct sdcardfsjb_inode_info), 0,
				  SLAB_RECLAIM_ACCOUNT, init_once);
	if (!sdcardfsjb_inode_cachep)
		err = -ENOMEM;
	return err;
}

/* sdcardfsjb inode cache destructor */
void sdcardfsjb_destroy_inode_cache(void)
{
	if (sdcardfsjb_inode_cachep)
		kmem_cache_destroy(sdcardfsjb_inode_cachep);
}

/*
 * Used only in nfs, to kill any pending RPC tasks, so that subsequent
 * code can actually succeed and won't leave tasks that need handling.
 */
static void sdcardfsjb_umount_begin(struct super_block *sb)
{
	struct super_block *lower_sb;

	lower_sb = sdcardfsjb_lower_super(sb);
	if (lower_sb && lower_sb->s_op && lower_sb->s_op->umount_begin)
		lower_sb->s_op->umount_begin(lower_sb);
}

static int sdcardfsjb_show_options(struct seq_file *m, struct vfsmount *mnt)
{
	struct sdcardfsjb_sb_info *sbi = SDCARDFSJB_SB(mnt->mnt_sb);
	struct sdcardfsjb_mount_options *opts = &sbi->options;

	if (opts->fs_low_uid != 0)
		seq_printf(m, ",uid=%u", opts->fs_low_uid);
	if (opts->fs_low_gid != 0)
		seq_printf(m, ",gid=%u", opts->fs_low_gid);

	return 0;
};

const struct super_operations sdcardfsjb_sops = {
	.put_super	= sdcardfsjb_put_super,
	.statfs		= sdcardfsjb_statfs,
	.remount_fs	= sdcardfsjb_remount_fs,
	.evict_inode	= sdcardfsjb_evict_inode,
	.umount_begin	= sdcardfsjb_umount_begin,
	.show_options	= sdcardfsjb_show_options,
	.alloc_inode	= sdcardfsjb_alloc_inode,
	.destroy_inode	= sdcardfsjb_destroy_inode,
	.drop_inode	= generic_delete_inode,
};
