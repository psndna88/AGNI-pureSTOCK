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

#ifndef _SDCARDFSJB_H_
#define _SDCARDFSJB_H_

#include <linux/dcache.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/mount.h>
#include <linux/namei.h>
#include <linux/seq_file.h>
#include <linux/statfs.h>
#include <linux/fs_stack.h>
#include <linux/magic.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/security.h>

/* the file system name */
#define SDCARDFSJB_NAME "sdcardfsjb"

/* sdcardfsjb root inode number */
#define SDCARDFSJB_ROOT_INO     1

/* useful for tracking code reachability */
#define UDBG printk(KERN_DEFAULT "DBG:%s:%s:%d\n", __FILE__, __func__, __LINE__)

#define SDCARDFSJB_DIRENT_SIZE 256

/* ######## ATTENTION ########
 * Old LOWER_FS_MIN_FREE_SIZE to 100MB.
 * Change New MIN_FREE_SIZE to 20MB cause of saving free memory.
 */
#define LOWER_FS_MIN_FREE_SIZE  (20*1048576UL) /* 20MB */

/* temporary static uid settings for development */ 
#define AID_ROOT             0  /* uid for accessing /mnt/sdcard & extSdcard */
#define AID_SDCARD_RW     1015  /* gid for accessing /mnt/sdcard & extSdcard */
#define AID_MEDIA_RW      1023  /* uid/gid for accessing underlying FS (=ext4)*/

/* for FAT emulation. Re-set uid, gid, and last 16 bytes of i_mode */
#define fix_mode(x)				\
	do {					\
		if (S_ISDIR(x)) 		\
			(x) = ((x) & S_IFMT) | 00775; 	\
		else					\
			(x) = ((x) & S_IFMT) | 00664; 	\
	} while (0)	

#define fix_fat_permission(x)		\
	do {					\
		(x)->i_uid = AID_ROOT;		\
		(x)->i_gid = AID_SDCARD_RW; 	\
		fix_mode((x)->i_mode);		\
	} while (0)

/* OVERRIDE_CRED() and REVERT_CRED() 
 * 	OVERRID_CRED() 
 * 		backup original task->cred
 * 		and modifies task->cred->fsuid/fsgid to specified value.
 *	REVERT_CRED()
 * 		restore original task->cred->fsuid/fsgid.
 * These two macro should be used in pair, and OVERRIDE_CRED() should be 
 * placed at the beginning of a function, right after variable declaration.
 */
#define OVERRIDE_CRED(sdcardfsjb_sbi) 		\
	const struct cred * old_cred;		\
	old_cred = override_fsids_jb(sdcardfsjb_sbi);	\
        if (!old_cred) { return -ENOMEM; }

#define OVERRIDE_CRED_PTR(sdcardfsjb_sbi) 	\
	const struct cred * old_cred;		\
	old_cred = override_fsids_jb(sdcardfsjb_sbi);	\
        if (!old_cred) { return ERR_PTR(-ENOMEM); }

#define REVERT_CRED()	 revert_fsids_jb(old_cred)

#define DEBUG_CRED()		\
	printk("KAKJAGI: %s:%d fsuid %d fsgid %d\n", 	\
		__FUNCTION__, __LINE__, 		\
		(int)current->cred->fsuid, 		\
		(int)current->cred->fsgid); 

struct sdcardfsjb_sb_info;
struct sdcardfsjb_mount_options;

/* Do not directly use this function. Use OVERRIDE_CRED() instead. */
const struct cred * override_fsids_jb(struct sdcardfsjb_sb_info* sbi);
/* Do not directly use this function, use REVERT_CRED() instead. */
void revert_fsids_jb(const struct cred * old_cred);

/* operations vectors defined in specific files */
extern const struct file_operations sdcardfsjb_main_fops;
extern const struct file_operations sdcardfsjb_dir_fops;
extern const struct inode_operations sdcardfsjb_main_iops;
extern const struct inode_operations sdcardfsjb_dir_iops;
extern const struct inode_operations sdcardfsjb_symlink_iops;
extern const struct super_operations sdcardfsjb_sops;
extern const struct dentry_operations sdcardfsjb_dops;
extern const struct address_space_operations sdcardfsjb_aops, sdcardfsjb_dummy_aops;
extern const struct vm_operations_struct sdcardfsjb_vm_ops;

extern int sdcardfsjb_init_inode_cache(void);
extern void sdcardfsjb_destroy_inode_cache(void);
extern int sdcardfsjb_init_dentry_cache(void);
extern void sdcardfsjb_destroy_dentry_cache(void);
extern int new_dentry_private_data_jb(struct dentry *dentry);
extern void free_dentry_private_data_jb(struct dentry *dentry);
extern struct dentry *sdcardfsjb_lookup(struct inode *dir, struct dentry *dentry,
				    struct nameidata *nd);
extern int sdcardfsjb_interpose(struct dentry *dentry, struct super_block *sb,
			    struct path *lower_path);

/* file private data */
struct sdcardfsjb_file_info {
	struct file *lower_file;
	const struct vm_operations_struct *lower_vm_ops;
};

/* sdcardfsjb inode data in memory */
struct sdcardfsjb_inode_info {
	struct inode *lower_inode;
	struct inode vfs_inode;
};

/* sdcardfsjb dentry data in memory */
struct sdcardfsjb_dentry_info {
	spinlock_t lock;	/* protects lower_path */
	struct path lower_path;
};

struct sdcardfsjb_mount_options {
	uid_t fs_low_uid;
	gid_t fs_low_gid;
};

/* sdcardfsjb super-block data in memory */
struct sdcardfsjb_sb_info {
	struct super_block *lower_sb;
	/* FIXME plz use following two field */
	uid_t fs_uid;
	gid_t fs_gid;
	unsigned short fs_fmask;
	unsigned short fs_dmask;
	struct sdcardfsjb_mount_options options;
};

/*
 * inode to private data
 *
 * Since we use containers and the struct inode is _inside_ the
 * sdcardfsjb_inode_info structure, SDCARDFSJB_I will always (given a non-NULL
 * inode pointer), return a valid non-NULL pointer.
 */
static inline struct sdcardfsjb_inode_info *SDCARDFSJB_I(const struct inode *inode)
{
	return container_of(inode, struct sdcardfsjb_inode_info, vfs_inode);
}

/* dentry to private data */
#define SDCARDFSJB_D(dent) ((struct sdcardfsjb_dentry_info *)(dent)->d_fsdata)

/* superblock to private data */
#define SDCARDFSJB_SB(super) ((struct sdcardfsjb_sb_info *)(super)->s_fs_info)

/* file to private Data */
#define SDCARDFSJB_F(file) ((struct sdcardfsjb_file_info *)((file)->private_data))

/* file to lower file */
static inline struct file *sdcardfsjb_lower_file(const struct file *f)
{
	return SDCARDFSJB_F(f)->lower_file;
}

static inline void sdcardfsjb_set_lower_file(struct file *f, struct file *val)
{
	SDCARDFSJB_F(f)->lower_file = val;
}

/* inode to lower inode. */
static inline struct inode *sdcardfsjb_lower_inode(const struct inode *i)
{
	return SDCARDFSJB_I(i)->lower_inode;
}

static inline void sdcardfsjb_set_lower_inode(struct inode *i, struct inode *val)
{
	SDCARDFSJB_I(i)->lower_inode = val;
}

/* superblock to lower superblock */
static inline struct super_block *sdcardfsjb_lower_super(
	const struct super_block *sb)
{
	return SDCARDFSJB_SB(sb)->lower_sb;
}

static inline void sdcardfsjb_set_lower_super(struct super_block *sb,
					  struct super_block *val)
{
	SDCARDFSJB_SB(sb)->lower_sb = val;
}

/* path based (dentry/mnt) macros */
static inline void pathcpy(struct path *dst, const struct path *src)
{
	dst->dentry = src->dentry;
	dst->mnt = src->mnt;
}
/* Returns struct path.  Caller must path_put it. */
static inline void sdcardfsjb_get_lower_path(const struct dentry *dent,
					 struct path *lower_path)
{
	spin_lock(&SDCARDFSJB_D(dent)->lock);
	pathcpy(lower_path, &SDCARDFSJB_D(dent)->lower_path);
	path_get(lower_path);
	spin_unlock(&SDCARDFSJB_D(dent)->lock);
	return;
}
static inline void sdcardfsjb_put_lower_path(const struct dentry *dent,
					 struct path *lower_path)
{
	path_put(lower_path);
	return;
}
static inline void sdcardfsjb_set_lower_path(const struct dentry *dent,
					 struct path *lower_path)
{
	spin_lock(&SDCARDFSJB_D(dent)->lock);
	pathcpy(&SDCARDFSJB_D(dent)->lower_path, lower_path);
	spin_unlock(&SDCARDFSJB_D(dent)->lock);
	return;
}
static inline void sdcardfsjb_reset_lower_path(const struct dentry *dent)
{
	spin_lock(&SDCARDFSJB_D(dent)->lock);
	SDCARDFSJB_D(dent)->lower_path.dentry = NULL;
	SDCARDFSJB_D(dent)->lower_path.mnt = NULL;
	spin_unlock(&SDCARDFSJB_D(dent)->lock);
	return;
}
static inline void sdcardfsjb_put_reset_lower_path(const struct dentry *dent)
{
	struct path lower_path;
	spin_lock(&SDCARDFSJB_D(dent)->lock);
	pathcpy(&lower_path, &SDCARDFSJB_D(dent)->lower_path);
	SDCARDFSJB_D(dent)->lower_path.dentry = NULL;
	SDCARDFSJB_D(dent)->lower_path.mnt = NULL;
	spin_unlock(&SDCARDFSJB_D(dent)->lock);
	path_put(&lower_path);
	return;
}

/* locking helpers */
static inline struct dentry *lock_parent(struct dentry *dentry)
{
	struct dentry *dir = dget_parent(dentry);
	mutex_lock_nested(&dir->d_inode->i_mutex, I_MUTEX_PARENT);
	return dir;
}

static inline void unlock_dir(struct dentry *dir)
{
	mutex_unlock(&dir->d_inode->i_mutex);
	dput(dir);
}


#if defined(LOWER_FS_MIN_FREE_SIZE)
/*
 * Return 1, if a disk has enough free space, otherwise 0.
 * We assume that any files can not be overwritten.
 */
static inline int check_min_free_space(struct dentry *dentry, size_t size, int dir)
{
	int err;
	struct path lower_path;
	struct kstatfs statfs;
	u64 avail;

	/* Get fs stat of lower filesystem. */
	sdcardfsjb_get_lower_path(dentry, &lower_path);
	err = vfs_statfs(&lower_path, &statfs);
	sdcardfsjb_put_lower_path(dentry, &lower_path);

	if (unlikely(err))
		return 0;

	/* Invalid statfs informations. */
	if (unlikely(statfs.f_bsize == 0))
		return 0;

	/* if you are checking directory, set size to f_bsize. */
	if (unlikely(dir))
		size = statfs.f_bsize;

	/* available size */
	avail = statfs.f_bavail * statfs.f_bsize;

	/* not enough space */
	if ((u64)size > avail)
		return 0;

	/* enough space */
	if ((avail - size) > LOWER_FS_MIN_FREE_SIZE)
		return 1;

	return 0;
}
#endif

#endif	/* not _SDCARDFSJB_H_ */
