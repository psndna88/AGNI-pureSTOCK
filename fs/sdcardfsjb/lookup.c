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
#include "linux/delay.h"

/* The dentry cache is just so we have properly sized dentries */
static struct kmem_cache *sdcardfsjb_dentry_cachep;

int sdcardfsjb_init_dentry_cache(void)
{
	sdcardfsjb_dentry_cachep =
		kmem_cache_create("sdcardfsjb_dentry",
				  sizeof(struct sdcardfsjb_dentry_info),
				  0, SLAB_RECLAIM_ACCOUNT, NULL);

	return sdcardfsjb_dentry_cachep ? 0 : -ENOMEM;
}

void sdcardfsjb_destroy_dentry_cache(void)
{
	if (sdcardfsjb_dentry_cachep)
		kmem_cache_destroy(sdcardfsjb_dentry_cachep);
}

void free_dentry_private_data_jb(struct dentry *dentry)
{
	if (!dentry || !dentry->d_fsdata)
		return;
	kmem_cache_free(sdcardfsjb_dentry_cachep, dentry->d_fsdata);
	dentry->d_fsdata = NULL;
}

/* allocate new dentry private data */
int new_dentry_private_data_jb(struct dentry *dentry)
{
	struct sdcardfsjb_dentry_info *info = SDCARDFSJB_D(dentry);

	/* use zalloc to init dentry_info.lower_path */
	info = kmem_cache_zalloc(sdcardfsjb_dentry_cachep, GFP_ATOMIC);
	if (!info)
		return -ENOMEM;

	spin_lock_init(&info->lock);
	dentry->d_fsdata = info;

	return 0;
}

static int sdcardfsjb_inode_test(struct inode *inode, void *candidate_lower_inode)
{
	struct inode *current_lower_inode = sdcardfsjb_lower_inode(inode);
	if (current_lower_inode == (struct inode *)candidate_lower_inode)
		return 1; /* found a match */
	else
		return 0; /* no match */
}

static int sdcardfsjb_inode_set(struct inode *inode, void *lower_inode)
{
	/* we do actual inode initialization in sdcardfsjb_iget */
	return 0;
}

static struct inode *sdcardfsjb_iget(struct super_block *sb,
				 struct inode *lower_inode)
{
	struct sdcardfsjb_inode_info *info;
	struct inode *inode; /* the new inode to return */
	int err;

	/* in order for FAT emulation */
	//struct sdcardfsjb_sb_info *sb_info = sb->s_fs_info;

	inode = iget5_locked(sb, /* our superblock */
			     /*
			      * hashval: we use inode number, but we can
			      * also use "(unsigned long)lower_inode"
			      * instead.
			      */
			     lower_inode->i_ino, /* hashval */
			     sdcardfsjb_inode_test,	/* inode comparison function */
			     sdcardfsjb_inode_set, /* inode init function */
			     lower_inode); /* data passed to test+set fxns */
	if (!inode) {
		err = -EACCES;
		iput(lower_inode);
		return ERR_PTR(err);
	}
	/* if found a cached inode, then just return it */
	if (!(inode->i_state & I_NEW))
		return inode;

	/* initialize new inode */
	info = SDCARDFSJB_I(inode);

	inode->i_ino = lower_inode->i_ino;
	if (!igrab(lower_inode)) {
		err = -ESTALE;
		return ERR_PTR(err);
	}
	sdcardfsjb_set_lower_inode(inode, lower_inode);

	inode->i_version++;

	/* use different set of inode ops for symlinks & directories */
	if (S_ISDIR(lower_inode->i_mode))
		inode->i_op = &sdcardfsjb_dir_iops;
	else if (S_ISLNK(lower_inode->i_mode))
		inode->i_op = &sdcardfsjb_symlink_iops;
	else
		inode->i_op = &sdcardfsjb_main_iops;

	/* use different set of file ops for directories */
	if (S_ISDIR(lower_inode->i_mode))
		inode->i_fop = &sdcardfsjb_dir_fops;
	else
		inode->i_fop = &sdcardfsjb_main_fops;

	inode->i_mapping->a_ops = &sdcardfsjb_aops;

	inode->i_atime.tv_sec = 0;
	inode->i_atime.tv_nsec = 0;
	inode->i_mtime.tv_sec = 0;
	inode->i_mtime.tv_nsec = 0;
	inode->i_ctime.tv_sec = 0;
	inode->i_ctime.tv_nsec = 0;

	/* properly initialize special inodes */
	if (S_ISBLK(lower_inode->i_mode) || S_ISCHR(lower_inode->i_mode) ||
	    S_ISFIFO(lower_inode->i_mode) || S_ISSOCK(lower_inode->i_mode))
		init_special_inode(inode, lower_inode->i_mode,
				   lower_inode->i_rdev);

	/* all well, copy inode attributes */
	fsstack_copy_attr_all(inode, lower_inode);
	fsstack_copy_inode_size(inode, lower_inode);

	fix_fat_permission(inode);

	unlock_new_inode(inode);
	return inode;
}

/*
 * Connect a sdcardfsjb inode dentry/inode with several lower ones.  This is
 * the classic stackable file system "vnode interposition" action.
 *
 * @dentry: sdcardfsjb's dentry which interposes on lower one
 * @sb: sdcardfsjb's super_block
 * @lower_path: the lower path (caller does path_get/put)
 */
int sdcardfsjb_interpose(struct dentry *dentry, struct super_block *sb,
		     struct path *lower_path)
{
	int err = 0;
	struct inode *inode;
	struct inode *lower_inode;
	struct super_block *lower_sb;

	lower_inode = lower_path->dentry->d_inode;
	lower_sb = sdcardfsjb_lower_super(sb);

	/* check that the lower file system didn't cross a mount point */
	if (lower_inode->i_sb != lower_sb) {
		err = -EXDEV;
		goto out;
	}

	/*
	 * We allocate our new inode below by calling sdcardfsjb_iget,
	 * which will initialize some of the new inode's fields
	 */

	/* inherit lower inode number for sdcardfsjb's inode */
	inode = sdcardfsjb_iget(sb, lower_inode);
	if (IS_ERR(inode)) {
		err = PTR_ERR(inode);
		goto out;
	}

	d_add(dentry, inode);

out:
	return err;
}

/*
 * Main driver function for sdcardfsjb's lookup.
 *
 * Returns: NULL (ok), ERR_PTR if an error occurred.
 * Fills in lower_parent_path with <dentry,mnt> on success.
 */
static struct dentry *__sdcardfsjb_lookup(struct dentry *dentry,
		struct nameidata *nd, struct path *lower_parent_path)
{
	int err = 0;
	struct vfsmount *lower_dir_mnt;
	struct dentry *lower_dir_dentry = NULL;
	struct dentry *lower_dentry;
	const char *name;
	struct nameidata lower_nd;
	struct path lower_path;
	struct qstr this;

	/* must initialize dentry operations */
	d_set_d_op(dentry, &sdcardfsjb_dops);

	if (IS_ROOT(dentry))
		goto out;

	name = dentry->d_name.name;

	/* now start the actual lookup procedure */
	lower_dir_dentry = lower_parent_path->dentry;
	lower_dir_mnt = lower_parent_path->mnt;

	/* Use vfs_path_lookup to check if the dentry exists or not */
#ifdef CONFIG_SDCARD_FS_CI_SEARCH
	err = vfs_path_lookup(lower_dir_dentry, lower_dir_mnt, name,
			LOOKUP_CASE_INSENSITIVE, &lower_nd);
#else
	err = vfs_path_lookup(lower_dir_dentry, lower_dir_mnt, name, 0,
			&lower_nd);
#endif

	/* no error: handle positive dentries */
	if (!err) {
		sdcardfsjb_set_lower_path(dentry, &lower_nd.path);
		err = sdcardfsjb_interpose(dentry, dentry->d_sb, &lower_nd.path);
		if (err) /* path_put underlying path on error */
			sdcardfsjb_put_reset_lower_path(dentry);
		goto out;
	}

	/*
	 * We don't consider ENOENT an error, and we want to return a
	 * negative dentry.
	 */
	if (err && err != -ENOENT)
		goto out;

	/* instatiate a new negative dentry */
	this.name = name;
	this.len = strlen(name);
	this.hash = full_name_hash(this.name, this.len);
	lower_dentry = d_lookup(lower_dir_dentry, &this);
	if (lower_dentry)
		goto setup_lower;

	lower_dentry = d_alloc(lower_dir_dentry, &this);
	if (!lower_dentry) {
		err = -ENOMEM;
		goto out;
	}
	d_add(lower_dentry, NULL); /* instantiate and hash */

setup_lower:
	lower_path.dentry = lower_dentry;
	lower_path.mnt = mntget(lower_dir_mnt);
	sdcardfsjb_set_lower_path(dentry, &lower_path);

	/*
	 * If the intent is to create a file, then don't return an error, so
	 * the VFS will continue the process of making this negative dentry
	 * into a positive one.
	 */
	if (nd) {
		if (nd->flags & (LOOKUP_CREATE|LOOKUP_RENAME_TARGET))
			err = 0;
	} else
		err = 0;

out:
	return ERR_PTR(err);
}

/* 
 * On success:
 * 	fills dentry object appropriate values and returns NULL. 
 * On fail (== error)
 * 	returns error ptr
 *
 * @dir : Parent inode. It is locked (dir->i_mutex)
 * @dentry : Target dentry to lookup. we should set each of fields.
 *	     (dentry->d_name is initialized already)
 * @nd : nameidata of parent inode 
 */
struct dentry *sdcardfsjb_lookup(struct inode *dir, struct dentry *dentry,
			     struct nameidata *nd)
{
	struct dentry *ret, *parent;
	struct path lower_parent_path;
	int err = 0;

	OVERRIDE_CRED_PTR(SDCARDFSJB_SB(dir->i_sb));

	parent = dget_parent(dentry);

	sdcardfsjb_get_lower_path(parent, &lower_parent_path);

	/* allocate dentry private data.  We free it in ->d_release */
	err = new_dentry_private_data_jb(dentry);
	if (err) {
		ret = ERR_PTR(err);
		goto out;
	}
	ret = __sdcardfsjb_lookup(dentry, nd, &lower_parent_path);
	if (IS_ERR(ret))
		goto out;
	if (ret) 
		dentry = ret;
	if (dentry->d_inode)
		fsstack_copy_attr_times(dentry->d_inode,
					sdcardfsjb_lower_inode(dentry->d_inode));
	/* update parent directory's atime */
	fsstack_copy_attr_atime(parent->d_inode,
				sdcardfsjb_lower_inode(parent->d_inode));

out:
	sdcardfsjb_put_lower_path(parent, &lower_parent_path);
	dput(parent);
	REVERT_CRED();
	return ret;
}
