#pragma once

/* Needed for compilation */

struct statfs {
    int f_type;    /* Type of filesystem (see below) */
    int f_bsize;   /* Optimal transfer block size */
    int f_blocks;  /* Total data blocks in filesystem */
    int f_bfree;   /* Free blocks in filesystem */
    int f_bavail;  /* Free blocks available to
                            unprivileged user */
    int f_files;   /* Total inodes in filesystem */
    int f_ffree;   /* Free inodes in filesystem */
    int f_fsid;    /* Filesystem ID */
    int f_namelen; /* Maximum length of filenames */
    int f_frsize;  /* Fragment size (since Linux 2.6) */
    int f_flags;   /* Mount flags of filesystem
                            (since Linux 2.6.36) */
    int f_spare[4];
                   /* Padding bytes reserved for future use */
};

extern "C" int statfs(const char *path, struct statfs *buf);