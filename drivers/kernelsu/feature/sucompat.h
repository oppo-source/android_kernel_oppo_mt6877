#ifndef __KSU_H_SUCOMPAT
#define __KSU_H_SUCOMPAT
#include <asm/ptrace.h>
#include <linux/types.h>
#include "compat/kernel_compat.h"

#ifdef KSU_COMPAT_USE_STATIC_KEY
extern struct static_key_true ksu_su_compat_enabled;
#else
extern bool ksu_su_compat_enabled;
#endif

void ksu_sucompat_init(void);
void ksu_sucompat_exit(void);

// Handler functions exported for hook_manager
int ksu_handle_faccessat(int *dfd, const char __user **filename_user, int *mode, int *__unused_flags);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0) && defined(CONFIG_KSU_SUSFS)
int ksu_handle_stat(int *dfd, struct filename **filename, int *flags);
#else
int ksu_handle_stat(int *dfd, const char __user **filename_user, int *flags);
#endif // #if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0) && defined(CONFIG_KSU_SUSFS)

int ksu_handle_execve(int *fd, const char *filename, void *argv, void *envp, int *flags);
int ksu_handle_execveat(int *fd, struct filename **filename_ptr, void *argv, void *envp, int *flags);
int ksu_handle_post_execve(int *fd, const char *filename, void *argv, void *envp, int *flags, int *retval);
int ksu_handle_post_execveat(int *fd, struct filename **filename_ptr, void *argv, void *envp, int *flags, int *retval);

#ifdef CONFIG_KSU_SUSFS
int ksu_handle_execveat_sucompat(int *fd, struct filename **filename_ptr, void *argv, void *envp, int *flags);
int ksu_handle_post_execveat_sucompat(int *fd, struct filename **filename_ptr, void *argv, void *envp, int *flags,
                                      int *retval);
#endif

#ifdef CONFIG_KSU_TRACEPOINT_HOOK
// WARNING! THERE HAVE TRYING TO CALL SYSCALL INTERNALLY
// ENSURE CALL IT ONLY IN TRACEPOINT SYSCALL REDIRECT
int ksu_handle_execve_sucompat_tp_internal(const char __user **filename_user, int orig_nr, const struct pt_regs *regs);
int ksu_handle_execveat_sucompat_tp_internal(const char __user **filename_user, int orig_nr,
                                             const struct pt_regs *regs);
#endif

#endif