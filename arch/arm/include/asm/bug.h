#ifndef _ASMARM_BUG_H
#define _ASMARM_BUG_H


#ifdef CONFIG_BUG
#ifdef CONFIG_DEBUG_BUGVERBOSE
extern void __bug(const char *file, int line) __attribute__((noreturn));

extern void dump_stack(void);

/* give file/line information */
/* #define BUG()		__bug(__FILE__, __LINE__) */
#define BUG() do { \
        dump_stack(); \
        panic("BUG!"); \
} while (0)

#else

/* this just causes an oops */
/*#define BUG()		(*(int *)0 = 0)*/
#define BUG() do { \
        dump_stack(); \
        panic("BUG!"); \
} while (0)


#endif

#define HAVE_ARCH_BUG
#endif

#include <asm-generic/bug.h>

#endif
