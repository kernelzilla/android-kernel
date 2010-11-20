/*
 * linux/include/linux/lttlite-events.h
 *
 * Copyright (C) 1999-2004 Karim Yaghmour (karim@opersys.com)
 * Copyright (C) 2004, 2005 - MontaVista Software, Inc. (source@mvista.com)
 *
 * This contains the event definitions for the Linux Trace Toolkit.
 *
 * This file is released  under the terms of the GNU GPL version 2.
 * This program  is licensed "as is" without any warranty of any kind,
 * whether express or implied.
 *
 */

#ifndef _LINUX_TRACE_H
#define _LINUX_TRACE_H
#ifdef __KERNEL__

#include <linux/sched.h>
#include <linux/unistd.h>
#include <linux/ioctl.h>

#define LTT_LITE_VERSION "LTT-LITE 1.0.1"
#define LTT_EV_FILE_SYSTEM_IOCTL __NR_ioctl
#define LTT_EV_SOFT_IRQ_SOFT_IRQ LTT_LITE_EV_SOFT_IRQ
#define LTT_EV_SOFT_IRQ_TASKLET_ACTION LTT_LITE_EV_TASKLET
#define LTT_EV_SOFT_IRQ_TASKLET_HI_ACTION LTT_LITE_EV_TASKLET_HI
#define LTT_LITE_MAX_LOG_STRING_SIZE 50
#define LTT_LITE_TASK_COMM_LEN 32
#define LTT_LITE_LOG_STRING(var) ltt_lite_log_string(var, sizeof(var) - 1)
/* Performance tracking commands*/
#define LL_CMD 'l'
#define LLEVENT     _IOW(LL_CMD, 0x01, unsigned int)
#define LLINFO      _IOW(LL_CMD, 0x02, char *)
#define ltt_ev(ID, DATA)
#define ltt_ev_process(ID, DATA1, DATA2)
#define ltt_ev_trap_entry(DATA1, DATA2)	ltt_lite_ev_trap_entry(DATA1, DATA2)
#define ltt_ev_trap_exit() ltt_lite_ev_trap_exit()
#define ltt_ev_irq_entry(ID, KERNEL) ltt_lite_int_entry((unsigned short)ID, \
				KERNEL)
#define ltt_ev_irq_exit() ltt_lite_int_exit((unsigned short)irq)
#define ltt_ev_schedchange(DATA) ltt_lite_ev_schedchange(DATA)
#define ltt_ev_soft_irq(ID, DATA) ltt_lite_log_softirq(ID, \
				LTT_LITE_EVENT_ENTER, DATA)
#define ltt_init_sched_event(DATA, OUT, IN) ltt_lite_init_sched_event(DATA, \
				OUT, IN)
#define ltt_ev_process_exit(DATA1, DATA2) ltt_lite_ev_process_exit()
#define ltt_lite_ev_process(DATA1, DATA2) ltt_lite_ev_log_process(DATA1, DATA2)

#define CONFIG_LTT_LITE_ANDROID_LOG

extern struct resource ltt_lite_res;

enum {
	TRAP_T_RETURN = -1,
	TRAP_T_PAGE_FAULT = 14,
	TRAP_T_BAD_DATA_ABOUT = 18,
};

enum {
	LTT_LITE_RUN_TIMER,
	LTT_LITE_RUN_HRT_EXP,
	LTT_LITE_RUN_HRT_PED,
};

/* process event enum */
enum {
	LTT_LITE_EV_PROCESS_TABLE,
	LTT_LITE_EV_PROCESS_FORK,
	LTT_LITE_EV_PROCESS_EXEC,
	LTT_LITE_EV_PROCESS_EXIT,
	LTT_LITE_EV_PROCESS_COMM_CHANGE,
};

/*
 * LTTLITE event enum, start from 1 is to keep consistent with deinition of
 * APLV.
 */
enum {
	LTT_LITE_EV_SYSCALL_ENTRY = 1,
	LTT_LITE_EV_SCHEDULE,
	LTT_LITE_EV_MEM_PROFILE,
	LTT_LITE_EV_PROCESS,
	LTT_LITE_EV_TRAP,
	LTT_LITE_EV_INT,
	LTT_LITE_EV_SOFT_IRQ,
	LTT_LITE_EV_TASKLET,
	LTT_LITE_EV_TASKLET_HI,
	LTT_LITE_EV_STRING,
	LTT_LITE_EV_SIG,
	LTT_LITE_EV_SIG_HANDLE,
	LTT_LITE_EV_TIMER,
	LTT_LITE_EV_TIMER_RUN,
	/* keep LTT_LITE_EV_REPORT as the last event type */
	LTT_LITE_EV_REPORT,
#ifdef CONFIG_LTT_LITE_ANDROID_LOG
	LTT_LITE_EV_ANDROID_LOG,      /* /dev/log/main  */
	LTT_LITE_EV_ANDROID_EVENTLOG, /* /dev/log/event */
	LTT_LITE_EV_ANDROID_RADIO,    /* /dev/log/radio */
	LTT_LITE_EV_PRINTK,
#endif
	/* the item below is used as array length */
	LTT_LITE_EV_LAST,
};

/* log event in/out enum */
enum {
	/* sign used to indicate return of a log event */
	LTT_LITE_EVENT_RETURN,
	/* sign used to indicate enter of a log event */
	LTT_LITE_EVENT_ENTER,
};

/*
 * log_ps_buffer size *MUST NOT* exceed
 * LTTLITE_RSVBUF_SZ definded in task.h
 */
struct log_ps_buffer{
	/* number of syscalls this task made */
	unsigned long syscalls;
	/* number of file group syscalls this task made*/
	unsigned long file_syscalls;
	/* Number of context switches of this task */
	unsigned long ctxt;
};

struct ltt_lite_schedchange {
	unsigned short opid;   /* switch-out pid */
	unsigned short out_state; /* process state */
	unsigned short oprio; /* priority of process*/
	unsigned short reserve;
	/* following elements are data about switch in process */
	unsigned short ipid;   /* switch-in pid */
	unsigned short iprio; /* priority of process */
};

static inline void ltt_lite_init_sched_event(
				struct ltt_lite_schedchange *sched_event,
				struct task_struct *task_out,
				struct task_struct *task_in)
{
	sched_event->opid = (u32)task_out->pid;
	sched_event->ipid  = (u32)task_in->pid;
	sched_event->oprio = task_out->prio;
	sched_event->iprio = task_in->prio;
	sched_event->out_state = task_out->state;
}

void ltt_lite_int_entry(unsigned short, char);
void ltt_lite_int_exit(unsigned short);
void ltt_lite_log_softirq(unsigned short, unsigned int, unsigned int);
void ltt_lite_ev_sig(unsigned short, unsigned short, unsigned short);
void ltt_lite_ev_handle_sig(unsigned short, unsigned short, unsigned long);
void ltt_lite_log_timer(struct timer_list *, unsigned short);
void ltt_lite_run_timer(unsigned short, unsigned long, unsigned long);
void ltt_lite_early_init(void);
void ltt_lite_ev_process_exit(void);
void ltt_lite_ev_log_process(int type, struct task_struct *p);
void ltt_lite_log_syscall(char sign, int scno);
void ltt_lite_ev_trap_entry(unsigned short trapid, unsigned long address);
void ltt_lite_ev_trap_exit(void);
void ltt_lite_ev_schedchange(struct ltt_lite_schedchange *);
void ltt_lite_printf(char *fmt, ...);
void ltt_lite_syscall_param(int scno, char *string, int size);
int ltt_lite_get_ms_time(struct timeval *ktv);
int ltt_lite_log_string(char *string, int size);
#ifdef CONFIG_LTT_LITE_ANDROID_LOG
int  ltt_lite_log_android(const struct iovec *iov,
						unsigned long nr_segs,
						char logchar);
void ltt_lite_log_printk(char *string, int size);
#endif

#endif /* __KERNEL__ */
#endif /* _LINUX_TRACE_H */
