// SPDX-License-Identifier: Apache-2.0
// Copyright 2014-2019 Peter A. Bigot

/** Stubs required for full newlib system support
 *
 * How newlib from GNU Toolchain for ARM Embedded works:
 * @li @c write() invokes @c _write_r() with the current reentrancy context;
 * @li @c _write_r() invokes @c _write() and copies errno appropriately;
 * @li @c _write() must be provided by something
 *
 * The set of functions and data objects corresponds to those provided
 * by newlib's <tt>-specs=nosys.specs</tt> option, which provides
 * default implementations when there is no system environment.  We're
 * not using that solution, because BSPACM does provide some pieces of
 * a system environment sometimes and I don't want to have to worry
 * about whose empty implementation gets found by the linker first.
 *
 * There are weak definitions for all functions here; these
 * implementations simply return an error.  The expectation is that
 * something (libbspacm or the user application) provides any required
 * non-trivial implementation.
 *
 * @file */

#include <errno.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/times.h>
#include <sys/types.h>

/* We're providing system call stub implementations here, so ensure we
 * have visible prototypes that match what newlib is expecting. */
#define _COMPILING_NEWLIB
#include <sys/unistd.h>

static char* env[] = { 0 };

__attribute__((__weak__))
char** environ = env;

__attribute__((__weak__))
int
_chown (const char* path,
        uid_t owner,
        gid_t group)
{
  errno = ENOSYS;
  return -1;
}

__attribute__((__weak__))
int
_close (int fd)
{
  errno = ENOSYS;
  return -1;
}

__attribute__((__weak__))
int
_execve (const char* filename,
         char* const argv[],
         char* const envp[])
{
  errno = ENOSYS;
  return -1;
}

__attribute__((__weak__))
pid_t
_fork (void)
{
  errno = ENOSYS;
  return -1;
}

__attribute__((__weak__))
int
_fstat (int fd,
        struct stat * buf)
{
  errno = ENOSYS;
  return -1;
}

__attribute__((__weak__))
pid_t
_getpid (void)
{
  errno = ENOSYS;
  return 1;
}

__attribute__((__weak__))
int
_gettimeofday (struct timeval * tv,
               struct timezone * tz)
{
  errno = ENOSYS;
  return -1;
}

__attribute__((__weak__))
int
_isatty (int fd)
{
  errno = ENOSYS;
  return -1;
}

__attribute__((__weak__))
int
_kill (pid_t pid,
       int sig)
{
  errno = ENOSYS;
  return -1;
}

__attribute__((__weak__))
int
_link (const char* oldpath,
       const char* newpath)
{
  errno = ENOSYS;
  return -1;
}

__attribute__((__weak__))
off_t
_lseek (int fd,
        off_t offset,
        int whence)
{
  errno = ENOSYS;
  return -1;
}

__attribute__((__weak__))
int
_open (const char* pathname,
       int flags)
{
  errno = ENOSYS;
  return -1;
}

#if !(ENABLE_CSTDIO - 0)
__attribute__((__weak__))
ssize_t
_read (int fd,
       void* buf,
       size_t count)
{
  errno = ENOSYS;
  return -1;
}
#endif /* ENABLE_CSTDIO */

__attribute__((__weak__))
ssize_t
_readlink (const char* path,
           char* buf,
           size_t bufsiz)
{
  errno = ENOSYS;
  return -1;
}

/* The weak default for sbrk is in newlib/sbrk.c */

__attribute__((__weak__))
int
_stat (const char* path,
       struct stat * buf)
{
  errno = ENOSYS;
  return -1;
}

__attribute__((__weak__))
int
_symlink (const char* oldpath,
          const char* newpath)
{
  errno = ENOSYS;
  return -1;
}

__attribute__((__weak__))
clock_t
_times (struct tms *buf)
{
  errno = ENOSYS;
  return -1;
}

__attribute__((__weak__))
int
_unlink (const char* pathname)
{
  errno = ENOSYS;
  return -1;
}

__attribute__((__weak__))
pid_t
_wait (int* status)
{
  errno = ENOSYS;
  return -1;
}

#if !(ENABLE_CSTDIO - 0)
__attribute__((__weak__))
ssize_t
_write (int fd,
        const void* buf,
        size_t count)
{
  errno = ENOSYS;
  return -1;
}
#endif /* ENABLE_CSTDIO */

__attribute__((__weak__))
void
_exit (int status)
{
  errno = ENOSYS;
  while (1);
}
