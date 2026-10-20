#ifndef __KSU_H_APP_PROFILE
#define __KSU_H_APP_PROFILE

#include "uapi/app_profile.h"
#include <asm/thread_info.h>

#define TIF_KSU_DISABLE_ESCAPE_WITH_ROOT (BITS_PER_LONG == 64 ? 63 : 31)

// Escalate current process to root with the appropriate profile
int escape_with_root_profile(void);

void disable_seccomp(void);
void escape_to_root_for_init(void);

#endif
