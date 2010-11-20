/*
 * ltt-lite - the core functions of ltt lite
 *
 * Copyright (C) Motorola 2006-2009
 */

/*
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 */


/*!
 * @defgroup USER_SPACE_LTTLITE_PROC_DOCS /proc/lttlite
 *
 * Provides an interface to control Linux Trace Toolkit Lite (LTT-LITE)
 * and to retrieve trace information created by it.
 * When enabled, LTT-LITE creates a trace of significant system events
 * (process switching by the Linux scheduler, system call entry and exit).
 * This trace is stored (in binary form) in a regular file.
 * This trace file can be processed to create a human-readable trace.
 * LTT-LITE also keeps track of all processes in the system. It does this even
 * when LTT-LITE is disabled.
 *
 * @section LttLiteInitProc User-space Device Names
 * This driver supports the following devices in <code>/proc</code>:<br>
 *
 * @htmlonly
 * <table cellpadding="2" cellspacing="1" width="100%">
 * <tr halign="left" bgcolor="#D0D0D0"><td><b>Device Name</b></td>
 * <td><b>Function</b></td></tr>
 * <tr halign="left" bgcolor="#F0F0F0"><td><code>/proc/lttlite/init</code></td>
 * <td>When written, enables LTT-LITE in a specified mode or disables LTT-LITE.
 * <br> When read, returns LTT-LITE working mode</td></tr>
 * <tr halign="left" bgcolor="#F0F0F0"><td><code>/proc/lttlite/config</code>
 * </td><td>When written, sets certain features of LTT-LITE.
 * <br> When read, returns the current config of LTT-LITE.</td></tr>
 * </table>
 * @endhtmlonly
 *
 * @example
 * -bash-2.05b# echo 15 > /proc/lttlite/init
 * @endexample
 * The command above starts scheduling logging, system call logging
 * memory profiling, and trap logging.
 * The types of events are defined in bit manner:
 * TRAP_LOGGING=8, MEMORY_PROFILING=4, SYSCALL_LOGGING=2, SCHEDULING_LOGGING=1.
 * INT_LOGGING=16, SOFTIRQ_LOGGING=32
 * A number between 0~15 corresponds to the various combination of
 * the four types of events.
 *
 * @example
 * -bash-2.05b# cat /proc/lttlite/init
 * @endexample
 * The command above read current log mode
 *
 * @example
 * -bash-2.05b# echo "memint 1" > /proc/lttlite/config
 * @endexample
 * The command above sets the interval second of memory profiling.
 * A number between 1~9 is valid. Its default value is 1.
 *
 * @example
 * -bash-2.05b# echo "logfil /home/dlog" >/proc/lttlite/config
 * @endexample
 * The command above sets the name of the file in which LTT-LITE stores the
 * trace it generates.
 * The LTT-LITE trace file defaults to <code>/home/ltt_lite_profilelog</code>.
 *
 * @example
 * -bash-2.05b# echo "sysftg 0" > /proc/lttlite/config
 * @endexample
 * The command above changes system call group id. A number between
 * 0~5 is valid.
 * <table cellpadding="2" cellspacing="1" width="100%">
 * <tr halign="left" bgcolor="#D0D0D0"><td><b>Group ID</b></td>
 * <td><b>Description</b></td><td><b>Syscalls included</b></td></tr>
 * <tr halign="left" bgcolor="#F0F0F0"><td><code>0</code></td>
 * <td>default, log all system calls</td><td>all</td></tr>
 * <tr halign="left" bgcolor="#F0F0F0"><td><code>1</code></td>
 * <td>file, contains system calls related to file operations</td>
 * <td>read, write, open, close, creat, link,
 * unlink, execve, chdir, mknod, chmod,
 * lchown16, stat, lseek, mount, oldumount,
 * fstat, utime, access, rename, mkdir,
 * rmdir, dup, pipe, acct, umount,
 * ioctl, fcntl, chroot, dup2, old_select,
 * symlink, lstat, readlink, uselib, swapon,
 * old_readdir, old_mmap, truncate, ftruncate,
 * fchmod, fchown16, statfs, fstatfs,
 * socketcall, newstat, newlstat, newfstat,
 * fsync, fchdir, llseek, getdents, select,
 * flock, readv, writev, fdatasync, poll,
 * pread64, pwrite64, chown16, getcwd,
 * sendfile, truncate64, ftruncate64, stat64,
 * lstat64, fstat64, lchown, fchown,
 * chown, getdents64, pivot_root, fcntl64,
 * readahead, setxattr, lsetxattr, fsetxattr,
 * getxattr, lgetxattr, listxattr, llistxattr,
 * removexattr, lremovexattr, fremovexattr, sendfile64
 * epoll_ctl, epoll_wait, statfs64, fstatfs64,
 * utimes</td></tr>
 * <tr halign="left" bgcolor="#F0F0F0"><td><code>2</code></td>
 * <td>process, contains system calls related to process operations</td>
 * <td>exit, fork, waitpid, execve, wait4, clone,
 * vfork, exit_group, waitid</td></tr>
 * <tr halign="left" bgcolor="#F0F0F0"><td><code>3</code></td>
 * <td>network, contains system calls related to network operations</td>
 * <td>sethostname, socketcall, setdomainname</td></tr>
 * <tr halign="left" bgcolor="#F0F0F0"><td><code>4</code></td>
 * <td>signal, contains system calls related to signal operations</td>
 * <td>pause, kill, signal, sigaction, sgetmask,
 * ssetmask, sigsuspend, sigpending, sigreturn,
 * sigprocmask, rt_sigreturn, rt_sigaction,
 * rt_sigprocmask, rt_sigpending, rt_sigtimedwait,
 * rt_sigqueueinfo, rt_sigsuspend, sigaltstack,
 * tkill, tgkill</td></tr>
 * <tr halign="left" bgcolor="#F0F0F0"><td><code>5</code></td>
 * <td>ipc, contains system calls related to ipc operations</td>
 * <td>mq_open, mq_unlink, mq_timedsend, mq_timedreceive,
 * mq_notify, mq_getsetattr</td></tr></table>
 *
 * @example
 * 1. -bash-2.05b# echo pidflt 152 > /proc/lttlite/config
 * 2. -bash-2.05b# echo pidflt -100 > /proc/lttlite/config
 * 3. -bash-2.05b# echo pidflt 152-162 > /proc/lttlite/config
 * 4. -bash-2.05b# echo pidflt 1000- > /proc/lttlite/config
 * 5. -bash-2.05b# echo pidflt 152 160-170 650-700 > /proc/lttlite/config
 * 6. -bash-2.05b# echo pidflt > /proc/lttlite/config
 * @endexample
 * The command above make ltt-lite log events only related
 * to process whose pid is selected.
 * <br>The 1st command only log process with pid 152.
 * <br>The 2nd command log processes with pid <= 100.
 * <br>The 3rd command log processes with 152 <= pid <= 162.
 * <br>The 4th command log processes with pid >= 1000.
 * <br>The 5th command log processes with pid 152, or
 * 160 <= pid <= 170, or 650 <= pid <= 700.
 * <br>The 6th command clear process filters.
 * <br>User must make sure the numbers are in an incremental
 * order, both between and inside rules, or the rule will
 * be rejected by ltt-lite.
 * <br>This feature currently applies to syscall logging, scheduling logging,
 * trap logging.
 * <br>It is better to use "cat config" to check if the
 * rules are set successfully.
 * <br>At most three rules are accepted. More rules are ignored.
 * <br>Interrupt filters are same as pid filter
 * @example
 * -bash-2.05b# echo priram 1 >/proc/lttlite/config
 * @endexample
 * The command above let ltt-lite use private ram as buffer.
 * @example
 * -bash-2.05b# echo priram 0 >/proc/lttlite/config
 * @endexample
 * @example
 * -bash-2.05b# echo ramfil 1 >/proc/lttlite/config
 * @endexample
 * @example
 * -bash-2.05b# echo ramfil 0 >/proc/lttlite/config
 * @endexample
 * The command above forbid ltt-lite use private ram as buffer.
 *
 * @example
 * -bash-2.05b# echo + >/proc/lttlite/init
 * @endexample
 * The command above append private ram content to a existed log file.
 * config->logfil is the file to be appended. This feature is used in private
 * ram mode to get a seamless log.
 *
 * @example
 * -bash-2.05b# echo filesz 5 >/proc/lttlite/config
 * @endexample
 * The command above make the log file maximal size to 5 MB. This is useful
 * for logging long time. The minimal size of the file is 2MB. If this value
 * equal zero, means there is no maximal size limit.
 *
 * @example
 * -bash-2.05b# cat /proc/lttlite/config
 * filesz:0M
 * priram: 0
 * ramfil: 0
 * memint: 1
 * sysftg: 0
 * logfil: /home/ltt_lite_profilelog
 * pidflt: 102, 120-130, 140-150
 * intflt:
 * @endexample
 * The command above lists configuration of LTT-LITE
 *
 * @example
 * Add "lttlite" to kernel boot command line; add
 * "echo 11 /proc/lttlite/init" to rcS.d/Start.sh, right below
 * "mountall.sh start".
 * @endexample
 * ltt-lite provides an early start feature.
 * You can get a continuous log starts from the time
 * when the init process is created. Writing early mode buffer
 * to file (on flash) takes a long time. This may cause the
 * later writing operation wait long and lose record when
 * buffer if full. So it is better to point your log file
 * to /tmp to avoid losing records.
 */
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/unistd.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/cpu.h>
#include <linux/init.h>
#include <linux/signal.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>
#include <linux/lttlite-events.h>
#include <linux/io.h>
#include <asm/mach/time.h>
#include <asm/setup.h>
#include <linux/page-flags.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/syscalls.h>
#include <linux/ioport.h>
#include <mach/system.h>

/* debug printk macro */
#ifdef LTT_LITE_DEBUG
#define DPRINT(args...) printk(args)
#else
#define DPRINT(args...)
#endif

#define UNUSED_PARAM(v) (void)(v)
/* command length when writing to config proc entry */
#define LTT_LITE_CMD_LEN 255

#define TIME_DELTA(a , b) ((unsigned long)((long)(a) - (long)(b)))

#define LTT_LITE_TRAP_RETURN -1
/* MIN_FILE_SIZE equal 2MB */
#define MIN_FILE_SIZE 2
#define SEEK_SET 0
/*
 * the file name for LTT LITE to store log data
 */
static char log_file[LTT_LITE_CMD_LEN] = "/data/ltt_lite_profilelog";

/* functions to set LTT version info */
static int set_ltt_version(void);
static int ltt_version_tag;

/* tag if buf has been overlapped in private ram mode */
static int is_buf_overlap;

/* page array for reserved pages, for free purpose */
static struct page **early_page_array, **page_array;
/* memory profiling interval, default to 1 second */
static int mem_profile_interval = 1;
static struct timer_list mem_profile_timer;
static void log_mem_status(unsigned long ignore);

/* the function that logs the top buf */
static void wq_log_top_half(struct work_struct *ignore);
/* the function that logs the bottom buf */
static void wq_log_bottom_half(struct work_struct *ignore);
/* the function that executes the initialization of workqueue */
static void wq_init(struct work_struct *ignore);
/* the function that executes close file and disable profile */
static void wq_close_file(struct work_struct *ignore);
/* declare the workqueues of the above four functions */
static DECLARE_WORK(wq_log_top_half_work, wq_log_top_half);
static DECLARE_WORK(wq_log_bottom_half_work, wq_log_bottom_half);
static DECLARE_WORK(wq_init_work, wq_init);
static DECLARE_WORK(wq_close_file_work, wq_close_file);

/* get a time stamp */
static inline void get_time_stamp(struct timeval *ktv);

/* alloc/free memory function */
static void free_reserved_pages(void *mem, struct page **pagearray,
				int pagecount);
static void *alloc_reserved_pages(struct page ***pagearray, int pagecount);

/* proc/lttlite for LTT LITE proc file directory */
static struct proc_dir_entry *proc_lttlite_entry;

/* record header used by all log data structures */
struct record_header_t{
	unsigned short type;
	unsigned short ssize;	/* header size + log struct size */
	unsigned long timestamp_sec;
	unsigned long timestamp_usec;
} record_header_t;

/* system call log data structure */
struct ltt_lite_syscalllog {
	struct record_header_t header;
	unsigned short pid;	/* pid of issue the syscall */
	short syscall_id;	/* syscall id */
};
/* kernel schedule log data structure */
struct ltt_lite_schedlog {
	struct record_header_t header;
	/* following elements are data about switch out process */
	unsigned short opid;	/* switch-out pid */
	unsigned short out_state;	/* process state */
	unsigned short oprio;	/* priority of process */
	unsigned short reserve;
	/* following elements are data about switch in process */
	unsigned short ipid;	/* switch-in pid */
	unsigned short iprio;	/* priority of process */
};

struct process_mem_log {
	struct record_header_t header;
	unsigned short pid;	/* switch-in pid */
	unsigned short reserve;
	unsigned long start_time_sec;	/* process start time */
	unsigned long start_time_usec;
	unsigned long vmsize;	/* total virtual memory size */
	unsigned long vmlck;	/* pages locked by kernel */
	unsigned long vmrss;	/* physical pages */
	unsigned long vmdata;	/* data section size */
	unsigned long vmstk;	/* stack section size */
	unsigned long vmexe;	/* text section size */
	unsigned long vmlib;	/* lib size */
	unsigned long vmrsv;	/* pages reserved by kernel */
	unsigned long vmshr;	/* pages could be shared */
	unsigned long min_flt;	/* page fault count do not block process */
	unsigned long maj_flt;	/* page fault count block process */
	unsigned long utime;
	unsigned long stime;
};

struct process_table_log {
	struct record_header_t header;
	unsigned short pid;
	unsigned short ppid;	/* parent pid */
	unsigned short tgid;	/* fork/clone record */
	unsigned long sub_type;
	char comm[LTT_LITE_TASK_COMM_LEN];
};
struct ltt_lite_trap_log {
	struct record_header_t header;
	unsigned short pid;
	unsigned short subtype;
};
/* signal log structures */
struct sig_send_log {
	struct record_header_t header;
	unsigned short s_pid;
	unsigned short r_pid;
	unsigned short sig;
	unsigned short reserve;
};
struct sig_handle_log {
	struct record_header_t header;
	unsigned short sig;
	unsigned short pid;
	unsigned long handler;
};
/* timer log data structure */
struct timer_log {
	struct record_header_t header;
	unsigned short pid;
	unsigned short sub_type;
	unsigned long exp;
	unsigned long ace;
	unsigned long fn;
};
struct timer_run_log {
	struct record_header_t header;
	unsigned short timer_type;
	unsigned short reserve;
	unsigned long fn;
	unsigned long data;
};
/* interrupt log data structure */
struct ltt_lite_intlog {
	struct record_header_t header;
	unsigned short intid;
	unsigned short subtype;
};

struct ltt_lite_soft_irq {
	struct record_header_t header;
	unsigned long sub_type;
	unsigned long data;
};
/*
 * ltt-lite may lose records due to buffer is exhausted
 * when it takes too much time to write file.
 * At least we should know whether or how many records are lost.
 * There is report for early mode and normal mode separately.
 * For early report, we write it to file right after written
 * the early buffer;
 * For normal report, we write it to the end of the log file,
 * just before closing the file.
 */
#define REPORT_ITEM_LEN (sizeof(unsigned long) * (LTT_LITE_EV_LAST - 2))
struct ltt_lite_report_event {
	struct record_header_t header;
	unsigned long reports[LTT_LITE_EV_LAST - 2];
};

#ifdef CONFIG_LTT_LITE_ANDROID_LOG
#define LTT_LITE_SMALL_BUFFER_SIZE 1024

static char ltt_lite_small_buffer[LTT_LITE_SMALL_BUFFER_SIZE];

/* this interface is for custom events used for the
 * Taskview-LTTLite android extensions.
 * */
struct ltt_lite_android_event {
	struct record_header_t header;
	unsigned short  pid;
	unsigned short  reserved;
	unsigned long   data;
};
#endif

static void commit_log(void *addr, int size, unsigned short type);

/* get /proc/<pid>/cmdline valude */
static int get_proc_pid_cmdline(struct task_struct *task, char *buffer);

/* save the early logged buffer to file */
static int write_early_buf(void *);

/* workqueue used by LTT LITE */
static struct workqueue_struct *ltt_lite_wq;
static long log_file_fd = -1;	/* logfile fd */

/* reserved pages for early logging */
static unsigned long ltt_lite_early_page_num = 1024;
/* reserve 128 pages for general logging */
#define LTT_LITE_PAGE_ORDER 7
static unsigned long early_unlogged_count[LTT_LITE_EV_LAST];

/*
 * buffer control struct
 * this stuct will be placed at the start of the buffer
 */
struct buf_info_t{
	unsigned long size;
	unsigned long buf_pos;
	unsigned long log_file_pos;
	char is_top;
	char buf_top_full;
	char buf_bottom_full;
	char reserved_2;
	unsigned long unlogged_count[LTT_LITE_EV_LAST];
};
static struct buf_info_t *buf_info;
#define LTT_LITE_PRI_RAM_SIZE SZ_64K
static unsigned long ltt_lite_buf_size;
static unsigned long ltt_lite_privateram_size;
static unsigned long ltt_lite_half_buf_size;
static unsigned long ltt_lite_file_size;
static unsigned long buf_empty_size;
static void ltt_lite_append_log(void);

static char *ltt_lite_buf;
static char *ltt_lite_privateram_virtaddr;
static char *ltt_lite_early_buf;
static unsigned long early_buf_pos;
/* indicate if ltt lite is enabled */
static bool ltt_lite_is_enabled;

static bool use_private_ram;
static long *pevent_count;
static char *pevent_table;
#define PEVENT_COUNT_MAX 1000
static bool use_private_ram_file;

/* bit definition of ltt_lite_mode */
enum ltt_lite_mode_t {
	LTT_LITE_MODE_DEFAULT = 0,
	LTT_LITE_MODE_SCHD = 1,
	LTT_LITE_MODE_SYSC = 1 << 1,
	LTT_LITE_MODE_MEM = 1 << 2,
	LTT_LITE_MODE_TRAP = 1 << 3,
	LTT_LITE_MODE_INT = 1 << 4,
	LTT_LITE_MODE_SOFTIRQ = 1 << 5,
	LTT_LITE_MODE_SIG = 1 << 6,
	LTT_LITE_MODE_TMR = 1 << 7,
#ifdef CONFIG_LTT_LITE_ANDROID_LOG
	LTT_LITE_MODE_PRINTK  = 1 << 8,
	LTT_LITE_MODE_ANDROID_RADIO = 1 << 9,
	LTT_LITE_MODE_ANDROID = 1 << 10,
#endif
};

/*indicate ltt lite working mode */
static enum ltt_lite_mode_t ltt_lite_mode;

#ifdef CONFIG_LTT_LITE_ANDROID_LOG
/* indicate Android message stream logigng mode
 * 0 = Log Android message via existing method and into the LTT-Lite log file
 * 1 = Log Android message into the LTT-Lite log file only
 */
static int android_logging_mode = 1;
#endif

#define MASK_ARRAY_LEN 18

enum syscall_group_t {
	SYSCALL_GROUP_ALL,
	SYSCALL_GROUP_FILE,
	SYSCALL_GROUP_PROCESS,
	SYSCALL_GROUP_NETWORK,
	SYSCALL_GROUP_SIGNAL,
	SYSCALL_GROUP_IPC
};

/* save the syscall mask group id */
static enum syscall_group_t syscall_mask_group_id;

/* save the code of current syscall groups */
/* we user MASK_ARRAY_LEN numbers to record bit mask map for system calls
   in each group. 16bits (from bit0 to bit15) are used in each number. */
static unsigned short ltt_lite_syscall_mask[SYSCALL_GROUP_IPC +
						1][MASK_ARRAY_LEN]
	= {
/* syscall group code for all syscalls */
/* when syscall_mask_group_id is SYSCALL_GROUP_ALL, we won't check
   this array, so keep it empty for SYSCALL_GROUP_ALL group is ok */
	{},
/* syscall group code for file group */
	{57208, 20589, 1986, 41176, 0, 63228, 7256, 64, 61472,
	 22, 256, 2288, 32894, 9744, 47998, 6144, 11264, 0},
/* syscall group code for process group */
	{134, 0, 0, 0, 0, 0, 0, 260, 0,
	 0, 0, 16384, 0, 0, 0, 256, 0, 256},
/* syscall group code for network group */
	{0, 0, 0, 0, 1024, 0, 64, 512, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0},
/* syscall group code for signal group */
	{0, 8192, 32, 1, 824, 0, 0, 16512, 0,
	 0, 40960, 1039, 0, 0, 16384, 0, 4096, 0},
/* syscall group code for ipc group */
	{0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 252}
};

#define BITS_IN_SHORT (sizeof(unsigned short) * 8)
/* max length of pid string */
#define PID_NUM_LEN 5
/* a fake syscall id to indicate return from syscall */
#define LTT_LITE_SYSCALL_RETURN_ID -1
/* define maximum pid filter rule number */
#define LTT_LITE_PID_FILTER_MAX 3
/* max length of output string when reading config entry */
#define OUT_STR_LEN (300 + LTT_LITE_PID_FILTER_MAX * 12)
/* config command string length */
#define CFG_SELECTOR_LEN 6

static bool early_enabled_mode;
static bool early_tracing_is_allowed;
/* variable used to record pid filter rule number (0~3) */
static unsigned short ltt_lite_pid_filter_num;
/* array stores the pid filter rules */
static unsigned short ltt_lite_pid_filter[LTT_LITE_PID_FILTER_MAX][2];

/* array stores the interrupt filter rules */
static unsigned short ltt_lite_int_filter_num;
static unsigned short ltt_lite_int_filter[LTT_LITE_PID_FILTER_MAX][2];

enum ltt_lite_config_cmd {
	NONCMD,			/* not valid command */
	CMD_LOG_FILE_NAME,	/* set log file name command */
	CMD_MEM_PROF_INTERVAL,	/* set memory profiling interval (seconds) */
	CMD_SYSCALL_FILTER_GROUP,	/* set system call filter group id */
	CMD_PID_FILTER,		/* set process id filter */
	CMD_PRIVATE_RAM,	/* use private ram as buffer */
	CMD_RAM_FILE,		/* set private ram and writing file mode */
	CMD_FILE_SIZE,		/* set file size */
	CMD_INT_FILTER,		/* set intterrupt number filter */
#ifdef CONFIG_LTT_LITE_ANDROID_LOG
	CMD_ANDROID_LOGMODE, /* set logging mode for Android streams */
#endif
};

/*
 * get kernel parameter for ltt-lite
 * if lttlite is given, ltt-lite will enter early started mode.
 * In this mode, ltt-lite start logging before process 1 is created,
 * and log records until the early buffer is full.
 * This buffer is written to file the first time ltt-lite is started
 * by echo a non-zero number to /proc/lttlite/init.
 * The early buffer can't stand too long, to avoid losing records,
 * we should start this echo as early as possible(e.g., in rcS.d/Start.sh,
 * after mountall.sh is called), and redirect log file to
 * /tmp/ to save time.
 */
static void __init ltt_lite_cmdline_setup(char **arg)
{
	int res, i;
	unsigned long pagenum;
	char string[5];

	for (i = 0; i < 5 && isxdigit((*arg)[i]); i++)
		string[i] = (*arg)[i];
	string[i] = '\0';

	res = strict_strtoul(string, 10, &pagenum);

	if (res)
		DPRINT(KERN_INFO   "Ltt-lite pagenum may be incorret\n");


	/* at most reserve 20M for bootup log */
	ltt_lite_early_page_num = min_t(unsigned long, pagenum, 5120);
	early_tracing_is_allowed = true;
	printk(KERN_ERR
		   "Ltt-lite early mode enabled: bootlog = %lu pages\n",
		   ltt_lite_early_page_num);
}

__early_param("bootlog=", ltt_lite_cmdline_setup);

struct resource ltt_lite_res = {
	.start = 0x0,
	.end = 0x0,
};

static int is_privateram_reserved(void)
{

	if (ltt_lite_res.start)
		return 1;
	else
		return 0;
}

static char *get_privateram_address(void)
{
	if (ltt_lite_res.start && !ltt_lite_privateram_virtaddr) {
		ltt_lite_privateram_virtaddr =
			(char *) ioremap(ltt_lite_res.start,
					 ltt_lite_privateram_size);
	}
	return ltt_lite_privateram_virtaddr;
}

static void release_privateram_address(void *addr)
{
	iounmap(addr);
	ltt_lite_privateram_virtaddr = NULL;
}

/*
 * setup private ram size and position for LTT-LITE
 */
static void __init ltt_lite_parser_mem(char **arg)
{
	unsigned long size, base;
	size = memparse(*arg, arg);
	if (**arg == '@') {
		base = memparse(*arg + 1, arg);
		ltt_lite_res.start = base + PHYS_OFFSET;
		ltt_lite_res.end = ltt_lite_res.start + size - 1;
		ltt_lite_privateram_size = size;
	}
	printk(KERN_DEBUG "ltt_lite_parser_mem:base 0x%lx, %lu\n",
		   (unsigned long) ltt_lite_res.start, size);
}

__early_param("lttpriram=", ltt_lite_parser_mem);

/*
 * pages used by ltt-lite should not be reclaimed for performance.
 * so wrap page malloc and page free with the following two functions.
 * every page got in alloc_reserved_pages is reserved and organized
 * into an array.
 *
 * allocate pages that can not be reclaimed
 * return NULL if failed, otherwise the address
 */
static void *alloc_reserved_pages(struct page ***pagearray, int pagecount)
{
	int n_pages;
	struct page **page_array;
	int page_array_size;
	int i;
	void *mem;
	n_pages = pagecount;
	page_array_size = n_pages * sizeof(struct page *);
	page_array = kmalloc(page_array_size, GFP_KERNEL);
	if (page_array == NULL)
		return NULL;

	memset(page_array, 0, page_array_size);
	for (i = 0; i < n_pages; i++) {
		page_array[i] = alloc_page(GFP_KERNEL);
		if (unlikely(!page_array[i])) {
			free_reserved_pages(NULL, page_array, i);
			return NULL;
		}
		SetPageReserved(page_array[i]);
	}
	mem = vmap(page_array, n_pages, GFP_KERNEL, PAGE_KERNEL);
	if (!mem) {
		free_reserved_pages(NULL, page_array, n_pages);
		return NULL;
	}
	memset(mem, 0, pagecount << PAGE_SHIFT);
	*pagearray = page_array;
	return mem;
}

/*
 * Free reserved pages
 */
static void free_reserved_pages(void *mem, struct page **pagearray,
				int pagecount)
{
	int j;
	if (mem)
		vunmap(mem);

	for (j = 0; j < pagecount; j++) {
		ClearPageReserved(pagearray[j]);
		__free_page(pagearray[j]);
	}
	kfree(pagearray);
}

/*
 * store process name information of all threads
 */
static int process_name_table_init(void)
{

	struct process_table_log process_log;
	struct task_struct *t, *p = &init_task;
	int res = 0, size = 0;
	char *buffer = "cmdline";

	memset(&process_log.header, 0, sizeof(struct record_header_t));
	process_log.header.type = LTT_LITE_EV_PROCESS;
	process_log.header.ssize = sizeof(process_log);
	process_log.sub_type = LTT_LITE_EV_PROCESS_TABLE;
	read_lock(&tasklist_lock);
	/*
	 * init_task is the idle process, do not need log.
	 * skip it.
	 */
	while ((p = next_task(p)) != &init_task) {
		t = p;
		do {
			process_log.pid = t->pid;
			process_log.ppid = t->parent->pid;
			buffer = "";
			process_log.tgid = t->tgid;

			/* update thread name  */
			res = get_proc_pid_cmdline(t, buffer);
			if (!res)
				buffer = t->comm;

			memset(process_log.comm, 0, sizeof(process_log.comm));
			size = strlen(buffer);
			if (size >= LTT_LITE_TASK_COMM_LEN)
				size = LTT_LITE_TASK_COMM_LEN - 1;
			memcpy(process_log.comm, buffer, size);

			commit_log(&process_log, sizeof(process_log),
				   LTT_LITE_EV_PROCESS);
			if (t->mm)
				t = next_thread(t);
			else
				break;
		} while (t != p);
	}
	read_unlock(&tasklist_lock);
	return 0;
}

/* workqueue open file and mark ltt lite enabled here */
static void wq_init(struct work_struct *ignore)
{
	int retval;
	char *buf;
	struct task_struct *ltt_kthread;
	struct sched_param param = {.sched_priority = 1 };
	UNUSED_PARAM(ignore);

	/*
	 * ltt_lite_is_enabled should not be set when enter this
	 * function.
	 */
	BUG_ON(ltt_lite_is_enabled);

	/*
	 * set the work queue to rt_priority
	 * low priority ittliter may not be scheduled in a long period,
	 * thus ltt-lite may lose records.
	 */
	set_user_nice(current, -20);
	retval = sched_setscheduler(current, SCHED_FIFO, &param);
	if (retval)
		printk(KERN_ERR "LTT-LITE: setscheduler(): %d.\n", retval);

	set_fs(get_ds());
	log_file_fd = sys_open((const char __user *)log_file,
					O_RDWR | O_CREAT | O_TRUNC,
					0600);
	if (log_file_fd  < 0) {
		printk(KERN_ERR "cannot open %s, %lu\n", log_file,
			   log_file_fd);
		return;
	}

	if (!ltt_lite_buf) {
		printk(KERN_DEBUG "Ltt-lite wq-int ltt-lite-buff true\n");
		if (!use_private_ram_file) {
			buf = alloc_reserved_pages(&page_array,
						   1 <<
						   LTT_LITE_PAGE_ORDER);
			if (!buf) {
				printk(KERN_ERR
					"cannot get pages for ltt_lite_buf\n");
			} else {
				ltt_lite_buf_size =
					PAGE_SIZE *
					(1 << LTT_LITE_PAGE_ORDER) -
					sizeof(struct buf_info_t);
			}
		} else {
			if (!is_privateram_reserved()) {
				printk(KERN_ERR
					   "no ltt-lite private ram!\n");
				log_file_fd = -1;
				return;
			}
			buf = get_privateram_address();
			ltt_lite_buf_size =
				ltt_lite_privateram_size
				- sizeof(struct buf_info_t);
		}
		if (!buf) {
			if (sys_close(log_file_fd))
				printk(KERN_ERR "%s: close error\n",
					   __func__);
			ltt_lite_buf_size = 0;
			return;
		}
		buf_info = (struct buf_info_t *) buf;
		ltt_lite_buf = buf + sizeof(struct buf_info_t);
		ltt_lite_half_buf_size = ltt_lite_buf_size / 2;
		memset(buf_info, 0, sizeof(struct buf_info_t));
		buf_info->size = ltt_lite_buf_size;
		buf_info->is_top = 1;
		buf_info->buf_pos = 0;
		buf_info->buf_top_full = 0;
		buf_info->buf_bottom_full = 0;
		buf_info->log_file_pos = ltt_lite_file_size ? 0 : -1;
	}

	ltt_lite_is_enabled = true;

	if (early_enabled_mode) {
		printk(KERN_DEBUG "LTT-LITE: wq-init early mode enabled\n");
		early_enabled_mode = false;

		ltt_kthread = kthread_run(write_early_buf, NULL, current->comm);
		if (!ltt_kthread)
			printk(KERN_ERR
				"LTT-LITE: create kernel thread failed\n");
	}

	set_ltt_version();
	process_name_table_init();
}

/*
 * write private ram to a new file, this function is used for priram mode
 */
static void ltt_lite_write_privateram(void)
{
	char *buf;
	int result;
	int buf_offset;
	int buf_write_size;
	long total_buf_head_size;

	set_fs(get_ds());
	if ((log_file_fd != -1) || ltt_lite_is_enabled) {
		printk(KERN_ERR "please disable ltt-lite first\n");
		return;
	}
	log_file_fd = sys_open((const char __user *) log_file,
				   O_RDWR | O_CREAT | O_TRUNC, 0600);
	if (log_file_fd < 0) {
		printk(KERN_ERR "cannot open %s, %ld\n", log_file,
			   log_file_fd);
		log_file_fd = -1;
		return;
	}
	if (!is_privateram_reserved()) {
		printk(KERN_ERR "no ltt-lite private ram!\n");
		if (sys_close(log_file_fd))
			printk(KERN_ERR "%s: close error\n", __func__);
		log_file_fd = -1;
		return;
	}
	buf = get_privateram_address();
	if (buf == NULL) {
		DPRINT(KERN_ALERT
			   "Error in mapping LTTLITE partition offset\n");
		if (sys_close(log_file_fd))
			printk(KERN_ERR "%s: close error\n", __func__);
		return;
	}
	buf_info = (struct buf_info_t *) buf;
	total_buf_head_size = sizeof(struct buf_info_t) +
					PEVENT_COUNT_MAX *
					sizeof(struct process_table_log);
	ltt_lite_buf_size = ltt_lite_privateram_size -
						total_buf_head_size;
	if (buf_info->size != ltt_lite_buf_size) {
		DPRINT(KERN_ALERT "Buf info not correct! %ld\n",
			   buf_info->size);
	}

	/* dump ram from three parts */
	if (is_buf_overlap) {
		/* part 1 buf table and process table */
		buf_offset = 0;
		buf_write_size = total_buf_head_size;
		result = sys_write(log_file_fd,
						buf + buf_offset,
						buf_write_size);
		if (result < 0)
			printk(KERN_ERR "%s: write part 1 error\n", __func__);
		/* part 2 log before overlalp */
		buf_offset = buf_info->buf_pos +
					total_buf_head_size;
		buf_write_size = ltt_lite_buf_size -
						buf_info->buf_pos;
		result = sys_write(log_file_fd,
						buf + buf_offset,
						buf_write_size);
		if (result < 0)
			printk(KERN_ERR "%s: write part 2 error\n", __func__);
		/* part 3 log after overlap */
		buf_offset = total_buf_head_size;
		buf_write_size = buf_info->buf_pos;
		result = sys_write(log_file_fd,
						buf + buf_offset,
						buf_write_size);
		if (result < 0)
			printk(KERN_ERR "%s: write part 3 error\n", __func__);
	} else {
		result = sys_write(log_file_fd, buf, ltt_lite_privateram_size);
		if (result < 0)
			printk(KERN_ERR "%s: write error\n", __func__);
	}

	if (sys_close(log_file_fd))
		printk(KERN_ERR "%s: close error\n", __func__);
	log_file_fd = -1;
	release_privateram_address(buf);
	ltt_lite_buf = NULL;
}

/*
 * used to append last part of log (in private ram) to an existed log file
 */
static void ltt_lite_append_log()
{
	char *buf;
	int result;
	struct ltt_lite_report_event lreport;

	set_fs(get_ds());
	if ((log_file_fd != -1) || ltt_lite_is_enabled) {
		printk(KERN_ERR "please disable ltt-lite first\n");
		return;
	}
	log_file_fd = sys_open((const char __user *) log_file,
				   O_WRONLY | O_APPEND, 0600);
	if (log_file_fd < 0) {
		printk(KERN_ERR "cannot open %s, %lu\n", log_file,
			   log_file_fd);
		log_file_fd = -1;
		return;
	}
	if (!is_privateram_reserved()) {
		printk(KERN_ERR "no ltt-lite private ram!\n");
		if (sys_close(log_file_fd))
			printk(KERN_ERR "%s: close error\n", __func__);
		log_file_fd = -1;
		return;
	}
	buf = get_privateram_address();

	buf_info = (struct buf_info_t *) buf;
	ltt_lite_buf = buf + sizeof(struct buf_info_t);
	ltt_lite_buf_size =
			ltt_lite_privateram_size - sizeof(struct buf_info_t);
	ltt_lite_half_buf_size = ltt_lite_buf_size / 2;
	/* Verify the private ram content is right */
	if ((buf_info->size != ltt_lite_buf_size) ||
		(buf_info->is_top
		 && (buf_info->buf_pos > ltt_lite_half_buf_size))
		|| (!buf_info->is_top
		&& (buf_info->buf_pos <= ltt_lite_half_buf_size))) {
		DPRINT(KERN_ALERT
			   "Buf info not correct, will not append!%lu, %lu\n",
			   buf_info->size, buf_info->buf_pos);
		if (sys_close(log_file_fd))
			printk(KERN_ERR "%s: close error\n", __func__);
		log_file_fd = -1;
		return;
	}
	if (buf_info->log_file_pos != -1) {
		result =
			sys_lseek(log_file_fd, buf_info->log_file_pos,
				  SEEK_SET);
		if (result < 0) {
			printk(KERN_ERR "%s: seek error - %d\n",
				   __func__, result);
			goto err_out;
		}
	}
	if (buf_info->is_top) {
		if (buf_info->buf_bottom_full) {
			result =
				sys_write(log_file_fd,
					  ltt_lite_buf +
					  ltt_lite_half_buf_size,
					  ltt_lite_half_buf_size);
			if (result < 0) {
				printk(KERN_ERR "%s: write error - %d\n",
					   __func__, result);
				goto err_out;
			}
		}
		result =
			sys_write(log_file_fd, ltt_lite_buf,
				  buf_info->buf_pos);
	} else {
		if (buf_info->buf_top_full) {
			result = sys_write(log_file_fd, ltt_lite_buf,
					   ltt_lite_half_buf_size);
			if (result < 0) {
				printk(KERN_ERR "%s: write error - %d\n",
					   __func__, result);
				goto err_out;
			}
		}
		result = sys_write(log_file_fd, ltt_lite_buf +
				   ltt_lite_half_buf_size,
				   buf_info->buf_pos -
				   ltt_lite_half_buf_size);
	}
	if (result < 0) {
		printk(KERN_ERR "%s: write error - %d\n", __func__,
			   result);
		goto err_out;
	}
	memset(&lreport.header, 0, sizeof(struct record_header_t));
	lreport.header.type = LTT_LITE_EV_REPORT;
	lreport.header.ssize = sizeof(struct ltt_lite_report_event);
	memcpy(&lreport.reports,
		   &buf_info->unlogged_count[LTT_LITE_EV_SYSCALL_ENTRY],
		   REPORT_ITEM_LEN);
	result =
		sys_write(log_file_fd, (char *) &lreport, sizeof(lreport));
	if (result < 0) {
		printk(KERN_ERR "%s: write report error - %d\n",
			   __func__, result);
		goto err_out;
	}
	memset(buf_info, 0, sizeof(struct buf_info_t));
err_out:
	result = sys_close(log_file_fd);
	if (result < 0)
		printk(KERN_ERR "%s: close error\n", __func__);
	log_file_fd = -1;
	release_privateram_address(buf);
	ltt_lite_buf = NULL;
}

/*
 * start ltt-lite early
 */
void ltt_lite_early_init(void)
{
	if (!early_tracing_is_allowed)
		return;
	ltt_lite_early_buf =
		(char *) alloc_reserved_pages(&early_page_array,
					  ltt_lite_early_page_num);
	if (!ltt_lite_early_buf) {
		printk(KERN_ERR "cannot get pages for early ltt-lite\n");
		return;
	}
	ltt_lite_mode = LTT_LITE_MODE_SCHD | LTT_LITE_MODE_SYSC
		| LTT_LITE_MODE_TRAP;
	early_enabled_mode = true;
}

/* workqueue writes the left log data before closes the file */
static void wq_close_file(struct work_struct *ignore)
{
	int result, size;
	char *start;
	struct ltt_lite_report_event lreport;
	UNUSED_PARAM(ignore);

	printk(KERN_DEBUG "LTT-LITE %s: wq_close_file enter\n", __func__);

	BUG_ON(log_file_fd < 0);
	/* write the log data has not be committed to log */
	if (buf_info->is_top) {
		start = ltt_lite_buf;
		size = buf_info->buf_pos;
	} else {
		start = ltt_lite_buf + ltt_lite_half_buf_size;
		size = buf_info->buf_pos - ltt_lite_half_buf_size;
	}

	result = sys_write(log_file_fd, start, size);
	if (result < 0)
		printk(KERN_ERR "%s: write error\n", __func__);
	/*
	 * write a report to the end of the log file
	 * only logs lost in normal ltt-lite mode
	 */
	memset(&lreport.header, 0, sizeof(struct record_header_t));
	lreport.header.type = LTT_LITE_EV_REPORT;
	lreport.header.ssize = sizeof(struct ltt_lite_report_event);
	memcpy(&lreport.reports,
		   &buf_info->unlogged_count[LTT_LITE_EV_SYSCALL_ENTRY],
		   REPORT_ITEM_LEN);

	result =
		sys_write(log_file_fd, (char *) &lreport, sizeof(lreport));
	if (result < 0)
		printk(KERN_ERR "%s: write error\n", __func__);
	memset(buf_info, 0, sizeof(struct buf_info_t));

	result = sys_close(log_file_fd);
	if (result < 0)
		printk(KERN_ERR "%s: close error\n", __func__);
	else
		log_file_fd = -1;
	/*
	 * ltt_lite_buf will not be freed twice for this work not been queued
	 * when ltt_lite_is_enabled is not set.
	 */
	if (!ltt_lite_buf)
		return;
	if (!use_private_ram_file) {
		free_reserved_pages(buf_info, page_array,
					1 << LTT_LITE_PAGE_ORDER);
	} else {
		release_privateram_address(ltt_lite_buf);
	}

	ltt_lite_buf = NULL;
}

/*
 * proc file init read function:
 */
static ssize_t
ltt_lite_proc_read_init(struct file *file, char __user *buffer,
			size_t buflen, loff_t *fpos)
{
	char buf[10];
	int len;

	UNUSED_PARAM(file);
	UNUSED_PARAM(buflen);
	if (*fpos)
		return 0;
	len = snprintf(buf, sizeof(buf), "%d\n", ltt_lite_mode);
	if (copy_to_user(buffer, buf, len))
		return -EFAULT;

	*fpos += len;
	return len;
}

static int init_private_ram(void)
{
	char *buf;
	if (!is_privateram_reserved()) {
		printk(KERN_ERR "no ltt-lite private ram!\n");
		return 0;
	}

	ltt_lite_buf_size = ltt_lite_privateram_size - sizeof(struct buf_info_t)
		- PEVENT_COUNT_MAX * sizeof(struct process_table_log);
	if (ltt_lite_buf_size < 0) {
		printk(KERN_ERR "ltt-lite private ram not enough!\n");
		return 0;
	}
	buf = get_privateram_address();

	buf_info = (struct buf_info_t *) buf;
	pevent_count = buf_info->unlogged_count;
	pevent_table = (char *) (buf + sizeof(struct buf_info_t));
	ltt_lite_buf = (char *) (buf + sizeof(struct buf_info_t) +
					PEVENT_COUNT_MAX *
					sizeof(struct process_table_log));

	memset(buf, 0, ltt_lite_privateram_size);
	buf_info->size = ltt_lite_buf_size;
	buf_info->is_top = 1;
	buf_info->buf_pos = 0;
	buf_info->buf_top_full = 0;
	buf_info->buf_bottom_full = 0;

	set_ltt_version();

	process_name_table_init();

	return 1;
}

/*
 * proc_init_char_command - deal with char command to
 * /proc/lttlite/init interface
 */
static inline int proc_init_char_command(char *str)
{
	if (*str == 'r' || *str == 'R') {
		/* maybe enable reset feature in future */
		/* arch_reset('h'); */
	} else if (*str == '+') {
		if (use_private_ram_file)
			ltt_lite_append_log();
		else if (use_private_ram)
			ltt_lite_write_privateram();

		return 1;
	}

	return 0;
}

/*
 * ltt_lite_proc_write_init - procfs write callback for init attr
 * write 1-7: start to enable logging
 * write 0: disable logging
 */
static int ltt_lite_proc_write_init(struct file *filp,
					const char *buffer, size_t count,
					loff_t *data)
{
	int buf, mode;
	int res, i;
	char string[5];
	UNUSED_PARAM(filp);
	UNUSED_PARAM(data);

	if (copy_from_user(string, buffer, 5))
		return -EFAULT;

	if (proc_init_char_command(string))
		return count;

	i = 0;
	while (isdigit(string[i]) && i < 5)
		i++;

	string[i] = '\0';

	res = strict_strtol(string, 0, (long *)&buf);
	if (res)
		return -EFAULT;

	mode =
		LTT_LITE_MODE_SCHD | LTT_LITE_MODE_SYSC | LTT_LITE_MODE_MEM |
		LTT_LITE_MODE_TRAP | LTT_LITE_MODE_INT | LTT_LITE_MODE_SOFTIRQ
		| LTT_LITE_MODE_SIG | LTT_LITE_MODE_TMR
#ifdef CONFIG_LTT_LITE_ANDROID_LOG
		| LTT_LITE_MODE_PRINTK
		| LTT_LITE_MODE_ANDROID
		| LTT_LITE_MODE_ANDROID_RADIO
#endif
		;

	if (buf < 0 || buf > mode)
		return count;

	/* stop profiling */
	if (!buf && ltt_lite_is_enabled) {
		if (use_private_ram)
			release_privateram_address(get_privateram_address
						   ());
		ltt_lite_is_enabled = false;
		if (ltt_lite_mode & LTT_LITE_MODE_MEM)
			del_timer_sync(&mem_profile_timer);

		if (log_file_fd >= 0 && ltt_lite_wq)
			queue_work(ltt_lite_wq, &wq_close_file_work);

	}
	/* start profiling */
	else if (buf && !ltt_lite_is_enabled) {
		if (use_private_ram) {
			if (init_private_ram())
				ltt_lite_is_enabled = 1;
			else
				return count;
		} else {
			if (!ltt_lite_wq) {
				/* enable ltt lite at first time */
				ltt_lite_wq =
					create_singlethread_workqueue
					("lttliter");
				BUG_ON(!ltt_lite_wq);
				queue_work(ltt_lite_wq, &wq_init_work);
			} else if (log_file_fd < 0) {
				queue_work(ltt_lite_wq, &wq_init_work);
			}
		}
		if (buf & LTT_LITE_MODE_MEM) {
			init_timer(&mem_profile_timer);
			mem_profile_timer.function = log_mem_status;
			mod_timer(&mem_profile_timer,
				  jiffies + mem_profile_interval * HZ);
		}
		/* recaculate file size limit */
		if (ltt_lite_file_size) {
			if (ltt_lite_file_size < MIN_FILE_SIZE)
				ltt_lite_file_size = MIN_FILE_SIZE;

			if (use_private_ram_file || use_private_ram) {
				if ((ltt_lite_file_size * SZ_1M) <
					(ltt_lite_privateram_size / 2)) {
					ltt_lite_file_size =
						ltt_lite_privateram_size / 2 /
						SZ_1M + 1;
				}
			}
		}
	}
	/* change profiling mode. Should deal with memory profiling timer */
	else if (buf && ltt_lite_is_enabled) {
		/* start memory profiling */
		if (!(ltt_lite_mode & LTT_LITE_MODE_MEM) &&
			(buf & LTT_LITE_MODE_MEM)) {
			init_timer(&mem_profile_timer);
			mem_profile_timer.function = log_mem_status;
			mod_timer(&mem_profile_timer, jiffies +
				  mem_profile_interval * HZ);
		}
		/* stop memory profiling */
		else if ((ltt_lite_mode & LTT_LITE_MODE_MEM) &&
			 !(buf & LTT_LITE_MODE_MEM)) {
			del_timer_sync(&mem_profile_timer);
		}
	}
	ltt_lite_mode = buf;

	return count;
}

static const struct file_operations proc_ltt_lite_init_operations = {
	.read = ltt_lite_proc_read_init,
	.write = ltt_lite_proc_write_init,
};

/*
 * open interface for proc file config
 */
static int ltt_lite_proc_config_open(struct inode *inode,
					 struct file *file)
{
	if ((file->f_mode & FMODE_WRITE) && !(inode->i_mode & S_IWUSR))
		return -EPERM;

	return 0;
}

/*
 * read interface for proc file config, show config info
 */
static ssize_t ltt_lite_proc_config_read(struct file *file,
					 char __user *buf, size_t count,
					 loff_t *ppos)
{
	char *config_buf;
	int i, len;
	int ret;

	UNUSED_PARAM(file);
	if (*ppos)
		return 0;

	config_buf = kmalloc(OUT_STR_LEN, GFP_KERNEL);
	if (!config_buf) {
		printk(KERN_ERR "%s: kmalloc failed\n", __func__);
		return 0;
	}

#ifdef CONFIG_LTT_LITE_ANDROID_LOG
	len = snprintf(config_buf, OUT_STR_LEN,
			   "filesz:%luM\npriram: %d\nramfil: %d\nmemint: \
			   %d\nsysftg: %d\nlogfil: %s\nlogmod: %d\npidflt: ",
				ltt_lite_file_size, use_private_ram,
				use_private_ram_file, mem_profile_interval,
				syscall_mask_group_id, log_file,
				android_logging_mode);
#else
	len = snprintf(config_buf, OUT_STR_LEN,
			   "filesz:%luM\npriram: %d\nramfil: %d\nmemint: \
			   %d\nsysftg: %d\nlogfil: %s\npidflt: ",
				ltt_lite_file_size, use_private_ram,
				use_private_ram_file, mem_profile_interval,
				syscall_mask_group_id, log_file);
#endif

	for (i = 0; i < ltt_lite_pid_filter_num; i++) {
		if (ltt_lite_pid_filter[i][0] == ltt_lite_pid_filter[i][1])
			len +=
				snprintf(config_buf + len, OUT_STR_LEN - len,
					 "%d ", ltt_lite_pid_filter[i][0]);
		else
			len +=
				snprintf(config_buf + len, OUT_STR_LEN - len,
					 "%d-%d ", ltt_lite_pid_filter[i][0],
					 ltt_lite_pid_filter[i][1]);
	}
	len += snprintf(config_buf + len, OUT_STR_LEN - len, "\nintflt:");
	for (i = 0; i < ltt_lite_int_filter_num; i++) {
		if (ltt_lite_int_filter[i][0] == ltt_lite_int_filter[i][1])
			len +=
				snprintf(config_buf + len, OUT_STR_LEN - len,
					 "%d ", ltt_lite_int_filter[i][0]);
		else
			len +=
				snprintf(config_buf + len, OUT_STR_LEN - len,
					 "%d-%d ", ltt_lite_int_filter[i][0],
					 ltt_lite_int_filter[i][1]);
	}
	len += snprintf(config_buf + len, OUT_STR_LEN - len, "\n");
	len = len < (int) count ? len : (int) count;

	ret = copy_to_user(buf, config_buf, len);
	kfree(config_buf);
	if (ret < 0)
		return -EFAULT;
	*ppos += len;

	return len;
}

/*
 * split string to substrings according to char pattern
 * deal with multiple characters of pattern
 * more parameters than max_param are ignored
 * the input string is modified
 * return value range from 0~max_param
 * when return 0, means there is no parameter
 */
static int split(char *string, char **index_array, char pattern,
		 int max_param)
{
	char *ptr;
	int count;

	/* thumb through the characters */
	for (ptr = string, count = 0; count < max_param; count++, ptr++) {
		/* find the start of substring */
		while (*ptr == pattern)
			ptr++;
		if (*ptr == '\0')
			break;
		*(index_array + count) = ptr;
		/* find the end of substring */
		while (*ptr != pattern && *ptr != '\0')
			ptr++;
		if (*ptr != '\0')
			*ptr = '\0';
		else {
			count++;
			break;
		}
	}

	return count;
}

/*
 * get pid number from string
 * check if the pid number is valid
 */
static int string2pid(char *string)
{
	int pidnum;
	int res;

	if (strlen(string) > PID_NUM_LEN)
		return -1;
	res = strict_strtol(string, 0, (long *)&pidnum);
	if (res)
		return -1;

	if (pidnum < 0 || pidnum > PID_MAX_DEFAULT)
		return -1;

	return pidnum;
}

/*
 * deal with pid filter command
 */
static void do_cmd_filter(char *params, unsigned short *array,
			  unsigned short *filter_num)
{
	char *char_sub_pos;
	int param_num = 0;
	int input_pid1 = -1;
	int input_pid2 = -1;
	char *substr_index[LTT_LITE_PID_FILTER_MAX];
	char *subpid_index[2];
	int i = 0;
	unsigned short temp_pid_filter[LTT_LITE_PID_FILTER_MAX][2];

	param_num =
		split(params, substr_index, ' ', LTT_LITE_PID_FILTER_MAX);
	if (!param_num) {
		/* clear filters if no parameter */
		*filter_num = 0;
		return;
	}

	for (i = 0; i < param_num; i++) {
		char_sub_pos = strchr(substr_index[i], (int) '-');
		/* more than one separator '-', invalid input */
		if (char_sub_pos != strrchr(substr_index[i], (int) '-'))
			break;
		/* pid filter rule "pidnum" */
		if (!char_sub_pos) {
			input_pid1 = string2pid(substr_index[i]);
			input_pid2 = input_pid1;
		}
		/* pid filter rule "-pidnum" */
		else if (char_sub_pos == substr_index[i]) {
			substr_index[i]++;
			input_pid1 = 0;
			input_pid2 = string2pid(substr_index[i]);
		}
		/* pid filter rule "pidnum-" */
		else if (char_sub_pos - substr_index[i] ==
			 strlen(substr_index[i]) - 1) {
			*char_sub_pos = '\0';
			input_pid1 = string2pid(substr_index[i]);
			input_pid2 = PID_MAX_DEFAULT;
		}
		/* pid filter rule "pidnum1-pidnum2" */
		else {
			split(substr_index[i], subpid_index, '-', 2);
			input_pid1 = string2pid(subpid_index[0]);
			input_pid2 = string2pid(subpid_index[1]);
		}
		/* string2pid do pid valid checking,
		   when invalid string, return a -1 on the pid */
		if (input_pid1 != -1 && input_pid2 != -1 &&
			input_pid1 <= input_pid2) {
			temp_pid_filter[i][0] = input_pid1;
			temp_pid_filter[i][1] = input_pid2;
		} else
			break;
	}
	/* do filter rules check */
	if (i == param_num) {
		for (i = 0; i < param_num - 1; i++) {
			if (temp_pid_filter[i][1] >=
				temp_pid_filter[i + 1][0])
				break;
		}
		if (i == param_num - 1) {
			memcpy(array, temp_pid_filter,
				   param_num * sizeof(unsigned short) * 2);
			*filter_num = param_num;
			return;
		}
	}
	DPRINT(KERN_NOTICE "Ltt-lite pid filter not set.\n");
}

/*
 * deal with command options according to command type
 */
static void do_cmd(char *cmd_line, int cmd_type)
{
	char *char_sub_pos;
	int res;
	int param_num = 0;
	int file_size = 0;

	switch (cmd_type) {
	case CMD_MEM_PROF_INTERVAL:
		param_num = split(cmd_line, &char_sub_pos, ' ', 1);
		if (param_num && strlen(char_sub_pos) == 1 &&
			*char_sub_pos >= '1' && *char_sub_pos <= '9') {
			mem_profile_interval = *char_sub_pos - '0';
		}
		break;
	case CMD_LOG_FILE_NAME:
		param_num = split(cmd_line, &char_sub_pos, ' ', 1);
		/* do not change log file name during logging */
		if (param_num && !ltt_lite_is_enabled &&
			strlen(char_sub_pos) < LTT_LITE_CMD_LEN) {
			memcpy(log_file, char_sub_pos,
				   strlen(char_sub_pos) + 1);
		}
		break;
	case CMD_SYSCALL_FILTER_GROUP:
		param_num = split(cmd_line, &char_sub_pos, ' ', 1);
		if (param_num && strlen(char_sub_pos) == 1 &&
			*char_sub_pos >= '0' && *char_sub_pos <= '5') {
			syscall_mask_group_id = *char_sub_pos - '0';
		}
		break;
	case CMD_PID_FILTER:
		do_cmd_filter(cmd_line,
				  (unsigned short *) ltt_lite_pid_filter,
				  &ltt_lite_pid_filter_num);
		break;
	case CMD_INT_FILTER:
		do_cmd_filter(cmd_line,
				  (unsigned short *) ltt_lite_int_filter,
				  &ltt_lite_int_filter_num);
		break;
		/*
		 * PRIRAM and RAMFIL can not be set in meanwhile
		 */
	case CMD_PRIVATE_RAM:
		if (use_private_ram_file) {
			printk(KERN_ERR "please disable RAMFIL first\n");
			break;
		}
		param_num = split(cmd_line, &char_sub_pos, ' ', 1);
		/* do not change buffer during logging */
		if (param_num && strlen(char_sub_pos) == 1 &&
			*char_sub_pos >= '0' && *char_sub_pos <= '1') {
			if (*char_sub_pos == '0' && use_private_ram == 1)
				ltt_lite_buf = NULL;

			use_private_ram = *char_sub_pos - '0';
		}
		break;
	case CMD_RAM_FILE:
		if (use_private_ram) {
			printk(KERN_ERR "please disable PRIRAM first\n");
			break;
		}
		param_num = split(cmd_line, &char_sub_pos, ' ', 1);
		/* do not change buffer during logging */
		if (param_num && strlen(char_sub_pos) == 1 &&
			*char_sub_pos >= '0' && *char_sub_pos <= '1') {
			use_private_ram_file = *char_sub_pos - '0';
		}
		break;
	case CMD_FILE_SIZE:
		param_num = split(cmd_line, &char_sub_pos, ' ', 1);
		/* do not change buffer during logging */
		if (param_num) {
			res = strict_strtoul(char_sub_pos,
							0,
							(long *) &file_size);
			if (res) {
				ltt_lite_file_size = 0;
				break;
			}

			if (file_size == 0) {
				ltt_lite_file_size = 0;
				break;
			}
			if (file_size < MIN_FILE_SIZE)
				file_size = MIN_FILE_SIZE;

			if (use_private_ram_file || use_private_ram) {
				if ((file_size * SZ_1M) <
					(ltt_lite_privateram_size / 2)) {
					file_size =
						ltt_lite_privateram_size / 2 /
						SZ_1M + 1;
				}
			}
			ltt_lite_file_size = file_size;
		}
		break;
#ifdef CONFIG_LTT_LITE_ANDROID_LOG
	case CMD_ANDROID_LOGMODE:
		param_num = split(cmd_line, &char_sub_pos, ' ', 1);
		android_logging_mode = *char_sub_pos - '0';
		break;
#endif
	}
}

/*
 * write interface for proc file config
 */
static ssize_t ltt_lite_proc_config_write(struct file *file,
					  const char __user *buf,
					  size_t count, loff_t *ppos)
{
	char cmd_line[LTT_LITE_CMD_LEN];
	int i = 0;
	enum ltt_lite_config_cmd cmd_type = NONCMD;

	UNUSED_PARAM(file);
	UNUSED_PARAM(ppos);

	if (ltt_lite_is_enabled) {
		printk(KERN_ERR "Please disable LTT-LITE first\n");
		return count;
	}

	if (count >= LTT_LITE_CMD_LEN)
		return 0;
	if (copy_from_user(cmd_line, buf, count))
		return -EFAULT;

	if (strncmp(cmd_line, "memint", CFG_SELECTOR_LEN) == 0)
		cmd_type = CMD_MEM_PROF_INTERVAL;
	else if (strncmp(cmd_line, "logfil", CFG_SELECTOR_LEN) == 0)
		cmd_type = CMD_LOG_FILE_NAME;
	else if (strncmp(cmd_line, "sysftg", CFG_SELECTOR_LEN) == 0)
		cmd_type = CMD_SYSCALL_FILTER_GROUP;
	else if (strncmp(cmd_line, "pidflt", CFG_SELECTOR_LEN) == 0)
		cmd_type = CMD_PID_FILTER;
	else if (strncmp(cmd_line, "priram", CFG_SELECTOR_LEN) == 0)
		cmd_type = CMD_PRIVATE_RAM;
	else if (strncmp(cmd_line, "ramfil", CFG_SELECTOR_LEN) == 0)
		cmd_type = CMD_RAM_FILE;
	else if (strncmp(cmd_line, "filesz", CFG_SELECTOR_LEN) == 0)
		cmd_type = CMD_FILE_SIZE;
	else if (strncmp(cmd_line, "intflt", CFG_SELECTOR_LEN) == 0)
		cmd_type = CMD_INT_FILTER;
#ifdef CONFIG_LTT_LITE_ANDROID_LOG
	else if (strncmp(cmd_line, "logmod", CFG_SELECTOR_LEN) == 0)
		cmd_type = CMD_ANDROID_LOGMODE;
#endif

	if (cmd_type != NONCMD) {
		for (i = 0; i < count; i++) {
			if (cmd_line[i] == '\n') {
				cmd_line[i] = '\0';
				break;
			}
		}
		cmd_line[count] = '\0';
		do_cmd(cmd_line + CFG_SELECTOR_LEN, cmd_type);
	}

	return count;
}

static const struct file_operations proc_ltt_lite_config_operations = {
	.open = ltt_lite_proc_config_open,
	.read = ltt_lite_proc_config_read,
	.write = ltt_lite_proc_config_write,
};

/* log all user applications' mem status */
static void log_mem_status(unsigned long ignore)
{
	struct task_struct *t, *p = &init_task;
	struct process_mem_log mem_log;
	struct mm_struct *mm;
	UNUSED_PARAM(ignore);

	if (!ltt_lite_is_enabled)
		return;

	read_lock(&tasklist_lock);
	while ((p = next_task(p)) != &init_task) {
		/*
		 * skip all kernel thread
		 */
		if (!p->mm)
			continue;
		memset((void *) &mem_log, 0,
			   sizeof(struct process_mem_log));
		mem_log.pid = p->pid;
		mem_log.start_time_sec = p->start_time.tv_sec;
		mem_log.start_time_usec = p->start_time.tv_nsec;
		mm = get_task_mm(p);
		mem_log.vmsize = mm->total_vm - mm->reserved_vm;
		mem_log.vmlck = mm->locked_vm;
		mem_log.vmrss = get_mm_rss(mm);
		mem_log.vmshr =
			mem_log.vmrss - get_mm_counter(mm, anon_rss);
		mem_log.vmdata =
			mm->total_vm - mm->shared_vm - mm->stack_vm;
		mem_log.vmstk = mm->stack_vm;
		mem_log.vmexe = (PAGE_ALIGN(mm->end_code) -
				 (mm->
				  start_code & PAGE_MASK)) >> PAGE_SHIFT;
		mem_log.vmlib = mm->exec_vm - mem_log.vmexe;
		mem_log.vmrsv = mm->reserved_vm;
		mmput(mm);
		t = p;
		do {
			mem_log.min_flt += t->min_flt;
			mem_log.maj_flt += t->maj_flt;
			mem_log.utime += t->utime;
			mem_log.stime += t->stime;
			t = next_thread(t);
		} while (t != p);

		commit_log(&mem_log, sizeof(mem_log),
			   LTT_LITE_EV_MEM_PROFILE);

	}
	read_unlock(&tasklist_lock);
	mod_timer(&mem_profile_timer, jiffies + mem_profile_interval * HZ);
}

/*
 * ltt_lite initialization function that initialize proc file for control
 * which is dependent on that kernel has already initialized file system
 * layer. So claim it as late_initcall
 */
int __init ltt_lite_init(void)
{
	struct proc_dir_entry *init_entry;
	struct proc_dir_entry *config_entry;

	proc_lttlite_entry = proc_mkdir("lttlite", NULL);
	if (proc_lttlite_entry == NULL) {
		printk(KERN_ERR "create proc lttlite directory error\n");
		return -ENOMEM;
	}
	init_entry = create_proc_entry("init", 0666, proc_lttlite_entry);
	if (init_entry == NULL) {
		remove_proc_entry("lttlite", NULL);
		printk(KERN_ERR "lttlite proc init create error\n");
		return -ENOMEM;
	}
	config_entry = create_proc_entry("config", 0666,
					 proc_lttlite_entry);
	if (config_entry == NULL) {
		remove_proc_entry("init", proc_lttlite_entry);
		remove_proc_entry("lttlite", NULL);
		printk(KERN_ERR "lttlite proc config create error\n");
		return -ENOMEM;
	}

	init_entry->proc_fops = &proc_ltt_lite_init_operations;
	config_entry->proc_fops = &proc_ltt_lite_config_operations;

	return 0;
}

late_initcall(ltt_lite_init);

/*
 * save the early logged buffer to file
 */
static int write_early_buf(void *ignore)
{
	int result;
	long early_log_file_fd;
	char early_log_file[LTT_LITE_CMD_LEN];
	struct ltt_lite_report_event lreport;
	UNUSED_PARAM(ignore);
	set_fs(get_ds());
	strncpy(early_log_file, log_file, LTT_LITE_CMD_LEN);
	strncat(early_log_file, ".early",
		LTT_LITE_CMD_LEN - strlen(log_file));
	early_log_file_fd = sys_open((const char __user *) early_log_file,
					 O_RDWR | O_CREAT | O_TRUNC, 0600);
	if (early_log_file_fd < 0) {
		printk(KERN_ERR "cannot open %s, %ld\n", early_log_file,
			   early_log_file_fd);
		early_log_file_fd = -1;
		return 0;
	}

	result = sys_write(early_log_file_fd, ltt_lite_early_buf,
			   early_buf_pos);
	/* write a report to the end of the early log */
	memset(&lreport.header, 0, sizeof(struct record_header_t));
	lreport.header.type = LTT_LITE_EV_REPORT;
	lreport.header.ssize = sizeof(struct ltt_lite_report_event);
	memcpy(&lreport.reports,
		   &early_unlogged_count[LTT_LITE_EV_SYSCALL_ENTRY],
		   REPORT_ITEM_LEN);
	result +=
		sys_write(early_log_file_fd, (char *) &lreport,
			  sizeof(lreport));
	if (result < 0)
		printk(KERN_ERR "%s: write error\n", __func__);
	sys_close(early_log_file_fd);

	free_reserved_pages(ltt_lite_early_buf, early_page_array,
				ltt_lite_early_page_num);
	ltt_lite_early_buf = NULL;

	return 0;
}

/*
 * save the top half buffer to file
 */
static void wq_log_top_half(struct work_struct *ignore)
{
	int result;
	UNUSED_PARAM(ignore);
	if (ltt_lite_file_size) {
		printk(KERN_DEBUG "bottom: buf_info->log_file_pos %lu\n",
			   buf_info->log_file_pos);
		if ((buf_info->log_file_pos + ltt_lite_half_buf_size) >=
			ltt_lite_file_size * SZ_1M) {
			buf_info->log_file_pos = 0;
			result =
				sys_lseek(log_file_fd, buf_info->log_file_pos,
					  SEEK_SET);
			if (result < 0) {
				printk(KERN_ERR "%s: seek error - %d\n",
					   __func__, result);
			}
		}
	}
	result = sys_write(log_file_fd, ltt_lite_buf,
			   ltt_lite_half_buf_size);
	if (result < 0)
		printk(KERN_ERR "%s: write error\n", __func__);
	if (ltt_lite_file_size)
		buf_info->log_file_pos += ltt_lite_half_buf_size;

	buf_info->buf_top_full = 0;
}

/*
 * save the bottom half buffer to file
 */
static void wq_log_bottom_half(struct work_struct *ignore)
{
	int result;
	UNUSED_PARAM(ignore);

	if (ltt_lite_file_size) {
		printk(KERN_DEBUG "bottom: buf_info->log_file_pos %lu\n",
			   buf_info->log_file_pos);
		if ((buf_info->log_file_pos + ltt_lite_half_buf_size -
			 buf_empty_size)
			>= ltt_lite_file_size * SZ_1M) {
			buf_info->log_file_pos = 0;
			result =
				sys_lseek(log_file_fd, buf_info->log_file_pos,
					  SEEK_SET);
			if (result < 0) {
				printk(KERN_ERR "%s: seek error - %d\n",
					   __func__, result);
			}
		}
	}
	result = sys_write(log_file_fd,
			   ltt_lite_buf + ltt_lite_half_buf_size,
			   ltt_lite_half_buf_size - buf_empty_size);
	if (result < 0)
		printk(KERN_ERR "%s: write error\n", __func__);
	if (ltt_lite_file_size) {
		buf_info->log_file_pos +=
			(ltt_lite_half_buf_size - buf_empty_size);
	}
	buf_info->buf_bottom_full = 0;
}

/*
 * count the missed events
 */
static inline void count_lost_event(unsigned short type)
{
	if (unlikely(early_enabled_mode))
		early_unlogged_count[type]++;
	else
		buf_info->unlogged_count[type]++;
}

/*
 * common entry for commit log data
 *
 * This is an algorithm which handles a buffer composite of
 * double buffer and circle buffer.
 * When log data size exceeds half size of the buffer, kernel
 * queues a work to lttlite workqueue and wakes up work thread
 * to write that half of the buffer.
 * To speed up logging, buf_empty_size bytes of bottom half buffer
 * are wasted without truncating one log, as schedule and syscall
 * events are frequent.
 * To make work thread as simple as writing file, this algorithm
 * declares two works for work queue, one writes top half while the
 * other writes bottom half. When complete writing file, the work
 * thread of workqueue release CPU immediately.
 */
static void commit_log(void *addr, int size, unsigned short type)
{
	struct record_header_t *header = addr;
	char in_irq_handler = 0;
	header->type = type;
	header->ssize = size;

	if (irqs_disabled())
		in_irq_handler = 1;
	if (!in_irq_handler)
		local_irq_disable();
	DPRINT(KERN_ERR "bj:%d,%d\n", type, in_irq_handler);
	if (early_enabled_mode) {
		if ((early_buf_pos + size) >
			((PAGE_SIZE * ltt_lite_early_page_num) - 1)) {
			count_lost_event(type);
			goto complete;
		}
		get_time_stamp((struct timeval *) &header->timestamp_sec);
		memcpy(ltt_lite_early_buf + early_buf_pos, addr, size);
		early_buf_pos += size;
		goto complete;
	} else if (use_private_ram) {
		/*
		 * purpose of assign "size" to 64 is to avoid log overlap
		 * issue when buffer overlap happening. get all log evnents
		 * to be align with biggest log record size
		 */
		if ((buf_info->buf_pos + 64) > (ltt_lite_buf_size - 1)) {
			buf_info->buf_pos = 0;
			is_buf_overlap = 1;
		}

		get_time_stamp((struct timeval *) &header->timestamp_sec);

		if (*pevent_count <= PEVENT_COUNT_MAX &&
			header->type == LTT_LITE_EV_PROCESS) {
			memcpy(pevent_table + *pevent_count *
				   sizeof(struct process_table_log), addr,
				   size);
			(*pevent_count)++;
		} else {
			memcpy(ltt_lite_buf + buf_info->buf_pos, addr,
				   size);
			buf_info->buf_pos += 64;
		}
		goto complete;
	}

	/*
	 * if log data is exceed the buffer size, write the log
	 * from the head of buffer, and queue the bottom half work
	 * to the work queue and wake up the thread to write bottom
	 * half buffer.
	 */
	if ((buf_info->buf_pos + size) > (ltt_lite_buf_size - 1)) {
		buf_empty_size = ltt_lite_buf_size - buf_info->buf_pos;
		if (buf_info->buf_top_full) {
			count_lost_event(type);
			goto complete;
		}
		buf_info->buf_pos = 0;
		buf_info->is_top = 1;
		buf_info->buf_bottom_full = 1;
		if (ltt_lite_wq)
			queue_work(ltt_lite_wq, &wq_log_bottom_half_work);
	}

	if (((buf_info->buf_pos + size) > ltt_lite_half_buf_size) &&
		buf_info->buf_bottom_full && buf_info->is_top) {
		count_lost_event(type);
		goto complete;
	}
	get_time_stamp((struct timeval *) &header->timestamp_sec);
	memcpy(ltt_lite_buf + buf_info->buf_pos, addr, size);
	buf_info->buf_pos += size;

	/*
	 * when log data size exceed half of buffer first time,
	 * queue top half work to work queue and wake up the thread
	 * to write top half buffer.
	 */
	if ((buf_info->buf_pos > ltt_lite_half_buf_size)
		&& buf_info->is_top) {
		buf_info->is_top = 0;
		buf_info->buf_top_full = 1;
		if (!ltt_lite_wq)
			goto complete;
		queue_work(ltt_lite_wq, &wq_log_top_half_work);
	}
	DPRINT(KERN_ERR "ej:%d\n", type);



complete:
	if (!in_irq_handler)
		local_irq_enable();
	return;
}

/*
 * log the process exit event, and link the process info
 * to the exit process list
 */
void ltt_lite_ev_process_exit(void)
{
	struct process_table_log process_log;
	int res = 0, size = 0;
	char *buffer = "cmdline";

	if (!(ltt_lite_is_enabled | early_enabled_mode))
		return;

	process_log.sub_type = LTT_LITE_EV_PROCESS_EXIT;
	process_log.pid = current->pid;
	process_log.ppid = current->parent->pid;
	process_log.tgid = current->tgid;

	res = get_proc_pid_cmdline(current, buffer);
	if (!res)
		buffer = current->comm;

	size = strlen(buffer);
	if (size >= LTT_LITE_TASK_COMM_LEN)
		size = LTT_LITE_TASK_COMM_LEN - 1;
	memset(process_log.comm, 0, sizeof(process_log.comm));
	memcpy(process_log.comm, buffer, size);

	commit_log(&process_log, sizeof(process_log), LTT_LITE_EV_PROCESS);
}

/*
 * get a time stamp
 */
static inline void get_time_stamp(struct timeval *ktv)
{
	ktime_get_ts((struct timespec *) ktv);
}

/*
 * use jiffies + system_timer->offset() to get the time stamp
 * can be used when can obtain xtime_lock
 * comment out because kernel has provided ktime_get_ts to get
 * time
 */
int ltt_lite_get_ms_time(struct timeval *ktv)
{
	ktime_get_ts((struct timespec *) ktv);

	return 0;
}
EXPORT_SYMBOL(ltt_lite_get_ms_time);

/*
 * check whether a pid is in pid filters
 * return 1 on true, 0 on false
 */
static inline int check_in_filter(unsigned short target,
				  unsigned short
				  array[LTT_LITE_PID_FILTER_MAX][2],
				  unsigned short num)
{
	int i;

	if (num == 0)
		return 1;
	for (i = 0; i < num; i++) {
		if (target >= array[i][0] && target <= array[i][1])
			return 1;
	}
	return 0;
}

/*
 * log the schedule change event
 */
void ltt_lite_ev_schedchange(struct ltt_lite_schedchange *schedchange)
{
	struct ltt_lite_schedlog sched_event;

	/*
	 * go on if ltt lite is enabled
	 * and ltt lite is in log scheduling mode
	 */
	if (!(ltt_lite_is_enabled | early_enabled_mode) ||
		!(ltt_lite_mode & LTT_LITE_MODE_SCHD))
		return;

	if (!check_in_filter
		(schedchange->ipid, ltt_lite_pid_filter,
		 ltt_lite_pid_filter_num)
		&& !check_in_filter(schedchange->opid, ltt_lite_pid_filter,
				ltt_lite_pid_filter_num))
		return;

	memcpy(&sched_event.opid, schedchange,
					sizeof(struct ltt_lite_schedchange));
	commit_log(&sched_event, sizeof(sched_event),
		   LTT_LITE_EV_SCHEDULE);
}

static inline int if_log_syscall(unsigned short syscall_pid,
				 short syscallid)
{
	if (!(ltt_lite_is_enabled | early_enabled_mode) ||
		!(ltt_lite_mode & LTT_LITE_MODE_SYSC))
		return 0;

	if (syscall_mask_group_id != SYSCALL_GROUP_ALL && syscallid >= 0) {
		/* use scno / 16 to get the index in the mask array,
		   use scno % 16 to get the bit in the mask variable */
		if ((ltt_lite_syscall_mask[syscall_mask_group_id]
			 [(int) (syscallid / BITS_IN_SHORT)]
			 & (1 << syscallid % BITS_IN_SHORT)) == 0)
			return 0;
	}

	if (!check_in_filter
		(syscall_pid, ltt_lite_pid_filter, ltt_lite_pid_filter_num))
		return 0;

	return 1;
}

/*
 * syscall log entry
 */
void ltt_lite_log_syscall(char sign, int scno)
{
	unsigned short syscall_pid = current->pid;
	struct ltt_lite_syscalllog syscall_event;
	short syscallid = scno;

	/*
	 * go on if ltt lite is enabled and
	 * ltt lite is in log system call mode
	 */
	if (!(if_log_syscall(syscall_pid, syscallid)))
		return;

	syscall_event.syscall_id = (sign == LTT_LITE_EVENT_ENTER) ?
		syscallid : LTT_LITE_SYSCALL_RETURN_ID;
	syscall_event.pid = syscall_pid;

	commit_log(&syscall_event, sizeof(syscall_event),
		   LTT_LITE_EV_SYSCALL_ENTRY);
}

/*
 * log process event log
 */
void ltt_lite_ev_log_process(int type, struct task_struct *p)
{
	struct process_table_log process_log;
	int res = 0, size = 0;
	char *buffer = "cmdline";

	if (!(ltt_lite_is_enabled | early_enabled_mode))
		return;

	process_log.sub_type = type;
	process_log.pid = p->pid;
	process_log.ppid = p->parent->pid;
	process_log.tgid = p->tgid;

	res = get_proc_pid_cmdline(p, buffer);
	if (!res)
		buffer = p->comm;

	size = strlen(buffer);
	if (size >= LTT_LITE_TASK_COMM_LEN)
		size = LTT_LITE_TASK_COMM_LEN - 1;
	memset(process_log.comm, 0, sizeof(process_log.comm));
	memcpy(process_log.comm, buffer, size);

	commit_log(&process_log, sizeof(process_log), LTT_LITE_EV_PROCESS);
}

/*
 * when trap is triggered, this log occurs.
 */
void ltt_lite_ev_trap_entry(unsigned short trapid, unsigned long address)
{
	struct ltt_lite_trap_log trap_log;
	unsigned short pid;
	UNUSED_PARAM(address);

	if (!(ltt_lite_is_enabled | early_enabled_mode) ||
		!(ltt_lite_mode & LTT_LITE_MODE_TRAP))
		return;
	pid = current->pid;
	if (!check_in_filter
		(pid, ltt_lite_pid_filter, ltt_lite_pid_filter_num))
		return;

	trap_log.subtype = trapid;
	trap_log.pid = pid;
	commit_log(&trap_log, sizeof(trap_log), LTT_LITE_EV_TRAP);
}

/*
 * when trap exit, this log is triggered.
 */
void ltt_lite_ev_trap_exit(void)
{
	struct ltt_lite_trap_log trap_log;
	unsigned short pid;

	if (!(ltt_lite_is_enabled | early_enabled_mode) ||
		!(ltt_lite_mode & LTT_LITE_MODE_TRAP))
		return;

	pid = current->pid;
	if (!check_in_filter
		(pid, ltt_lite_pid_filter, ltt_lite_pid_filter_num))
		return;

	trap_log.pid = pid;
	trap_log.subtype = LTT_LITE_TRAP_RETURN;
	commit_log(&trap_log, sizeof(trap_log), LTT_LITE_EV_TRAP);
}

/*
 * log interrupt entry
 */
void ltt_lite_int_entry(unsigned short intid, char kmod)
{
	struct ltt_lite_intlog int_log;

	if (!(ltt_lite_is_enabled | early_enabled_mode)
		|| !(ltt_lite_mode & LTT_LITE_MODE_INT))
		return;
	if (!check_in_filter
		(intid, ltt_lite_int_filter, ltt_lite_int_filter_num))
		return;

	int_log.intid = intid;
	int_log.subtype = kmod ? (LTT_LITE_EVENT_ENTER |
				  (1 << 15)) : LTT_LITE_EVENT_ENTER;
	commit_log(&int_log, sizeof(int_log), LTT_LITE_EV_INT);
}

/*
 * log interrup exit
 */
void ltt_lite_int_exit(unsigned short intid)
{
	struct ltt_lite_intlog int_log;

	if (!(ltt_lite_is_enabled | early_enabled_mode) ||
		!(ltt_lite_mode & LTT_LITE_MODE_INT))
		return;

	if (!check_in_filter
		(intid, ltt_lite_int_filter, ltt_lite_int_filter_num))
		return;

	int_log.intid = intid;
	int_log.subtype = LTT_LITE_EVENT_RETURN;
	commit_log(&int_log, sizeof(int_log), LTT_LITE_EV_INT);
}

void ltt_lite_log_softirq(unsigned short type, unsigned int sub_type,
			  unsigned int data)
{
	struct ltt_lite_soft_irq soft_irq_log;
	if (!(ltt_lite_is_enabled | early_enabled_mode)
		|| !(ltt_lite_mode & LTT_LITE_MODE_SOFTIRQ))
		return;

	soft_irq_log.sub_type = sub_type, soft_irq_log.data = data;

	commit_log(&soft_irq_log, sizeof(soft_irq_log), type);
}

/*
 * this interface is for some user to log special log.
 * This log is logged when ltt-lite is enabled.
 */
int ltt_lite_log_string(char *string, int size)
{
	char userinfo[sizeof(struct record_header_t) +
			  LTT_LITE_MAX_LOG_STRING_SIZE];

	/* we don't need ioctl parameters on bootup log */
	if (!(ltt_lite_is_enabled | early_enabled_mode))
		return 0;
	memset(userinfo, 0, sizeof(userinfo));
	if (size < LTT_LITE_MAX_LOG_STRING_SIZE) {
		memcpy(userinfo + sizeof(struct record_header_t), string, size);
		commit_log(userinfo, sizeof(struct record_header_t) + size,
			   LTT_LITE_EV_STRING);
	} else {
		memcpy(userinfo + sizeof(struct record_header_t), string,
			   LTT_LITE_MAX_LOG_STRING_SIZE);
		commit_log(userinfo, sizeof(userinfo), LTT_LITE_EV_STRING);
	}
	return 0;
}
EXPORT_SYMBOL(ltt_lite_log_string);

/*
 * printf-style interface to user logging
 *
 * @param fmt	printf-style format string
 */
void ltt_lite_printf(char *fmt, ...)
{
	va_list fmtargs;
	char buffer[LTT_LITE_MAX_LOG_STRING_SIZE];

	if (!(ltt_lite_is_enabled | early_enabled_mode))
		return;

	va_start(fmtargs, fmt);
	vsnprintf(buffer, sizeof(buffer) - 1, fmt, fmtargs);
	va_end(fmtargs);

	ltt_lite_log_string(buffer, LTT_LITE_MAX_LOG_STRING_SIZE);
}
EXPORT_SYMBOL(ltt_lite_printf);

void ltt_lite_syscall_param(int scno, char *string, int size)
{
	unsigned short syscall_pid = current->pid;
	short syscallid = scno;

	/*
	 * go on if ltt lite is enabled (non-early mode)and
	 * ltt lite is in log system call mode
	 * for early log mode, we only let __NR_lttlite go through
	 */
	if (!if_log_syscall(syscall_pid, syscallid)
		|| (early_enabled_mode && (scno != __NR_lttlite)))
		return;

	ltt_lite_log_string(string, size);
}

void ltt_lite_ev_sig(unsigned short s_pid, unsigned short r_pid,
			 unsigned short sig)
{
	struct sig_send_log slog;

	if (!(ltt_lite_is_enabled | early_enabled_mode) ||
		!(ltt_lite_mode & LTT_LITE_MODE_SIG))
		return;

	slog.s_pid = s_pid;
	slog.r_pid = r_pid;
	slog.sig = sig;
	commit_log(&slog, sizeof(slog), LTT_LITE_EV_SIG);
}

void ltt_lite_ev_handle_sig(unsigned short pid, unsigned short sig,
				unsigned long handler)
{
	struct sig_handle_log slog;

	if (!(ltt_lite_is_enabled | early_enabled_mode) ||
		!(ltt_lite_mode & LTT_LITE_MODE_SIG))
		return;

	slog.sig = sig;
	slog.pid = pid;
	slog.handler = handler;
	commit_log(&slog, sizeof(slog), LTT_LITE_EV_SIG_HANDLE);
}

void ltt_lite_log_timer(struct timer_list *timer, unsigned short type)
{
	struct timer_log tlog;

	if (!(ltt_lite_is_enabled | early_enabled_mode) ||
		!(ltt_lite_mode & LTT_LITE_MODE_TMR))
		return;

	tlog.pid = current->pid;
	tlog.sub_type = type;
	tlog.exp = timer->expires;
	/* tlog.ace = timer->arch_cycle_expires; */
	tlog.ace = 0;
	tlog.fn = (unsigned long) (timer->function);
	commit_log(&tlog, sizeof(tlog), LTT_LITE_EV_TIMER);
}

void ltt_lite_run_timer(unsigned short timer_type, unsigned long fn,
			unsigned long data)
{
	struct timer_run_log tlog;

	if (!(ltt_lite_is_enabled | early_enabled_mode) ||
		!(ltt_lite_mode & LTT_LITE_MODE_TMR))
		return;

	tlog.timer_type = timer_type;
	tlog.fn = fn;
	tlog.data = data;
	commit_log(&tlog, sizeof(tlog), LTT_LITE_EV_TIMER_RUN);
}

asmlinkage long sys_lttlite(unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case LLEVENT:{
			char ltt_string[LTT_LITE_MAX_LOG_STRING_SIZE];

			snprintf(ltt_string, LTT_LITE_MAX_LOG_STRING_SIZE,
				 "User event:pid=%d,eventid=%d\n",
				 current->pid, (unsigned int) arg);
			ltt_lite_syscall_param(__NR_lttlite, ltt_string,
						   strlen(ltt_string));

			break;
		}
	case LLINFO:{
			char ltt_string[LTT_LITE_MAX_LOG_STRING_SIZE];
			int len;

			len =
				strncpy_from_user(ltt_string, (char *) arg,
						  LTT_LITE_MAX_LOG_STRING_SIZE
						  - 1);
				if (len < 0)
					return -EFAULT;

			ltt_string[LTT_LITE_MAX_LOG_STRING_SIZE - 1] =
				'\0';

			ltt_lite_log_string(ltt_string, len);

			break;
		}
	default:
		return -EPERM;
	}

	return 0;
}
EXPORT_SYMBOL(sys_lttlite);

/*
 * get value of /proc/<pid>/cmdline
 */
static int get_proc_pid_cmdline(struct task_struct *task, char *buffer)
{
	unsigned int len;
	struct mm_struct *mm = get_task_mm(task);
	int res = 0;

	if (!mm)
		goto out;
	if (!mm->arg_end)
		goto out_mm;

	len = mm->arg_end - mm->arg_start;

	if (len > PAGE_SIZE)
		len = PAGE_SIZE;

	res = access_process_vm(task, mm->arg_start, buffer, len, 0);

	if (res > 0 && buffer[res - 1] != '\0' && len < PAGE_SIZE) {
		len = strnlen(buffer, res);
		if (len < res)
			res = len;
		else {
			len = mm->env_end - mm->env_start;
			if (len > PAGE_SIZE - res)
				len = PAGE_SIZE - res;
				res += access_process_vm(task,
							mm->env_start,
							buffer+res, len, 0);
			res = strnlen(buffer, res);
		}
	}

out_mm:
	mmput(mm);
out:
	return res;
}

/*
 * set Ltt-Lite version
 */
static int set_ltt_version(void)
{
	char ltt_string[LTT_LITE_MAX_LOG_STRING_SIZE];
	int len;

	if (ltt_version_tag)
		ltt_version_tag++;
	else {
		/* record LTT-Lite version in the 1st record */
		snprintf(ltt_string,
			LTT_LITE_MAX_LOG_STRING_SIZE,
			"%s\n",
			LTT_LITE_VERSION);
		len = strlen(ltt_string);

		ltt_lite_log_string(ltt_string, len);
		ltt_version_tag++ ;
	}

	return 0;
}

#ifdef CONFIG_LTT_LITE_ANDROID_LOG
/*
 * This interface is for logging custom LTT-lite events from
 * Android userspace (/dev/log/xxx), for graphical interpretation
 * by the Taskview postprocessing tool.
 */
int ltt_lite_log_android(const struct iovec *iov,
						unsigned long nr_segs,
						char logchar)
{
	struct ltt_lite_android_event *record;
	char *recptr;
	int   reclen;
	int   type;
	int   vecs;

	if (!ltt_lite_is_enabled)
		return 0;

	/* Select record type based on unique log name character */
	switch (logchar) {
			/* /dev/log/main   = android.util.Log      */
	case 'm':
		type = LTT_LITE_EV_ANDROID_LOG;
		break;

	/* /dev/log/events = android.util.EventLog */
	case 'e':
		type = LTT_LITE_EV_ANDROID_EVENTLOG;
		break;

	/* /dev/log/radio                          */
	case 'r':
		type = LTT_LITE_EV_ANDROID_RADIO;
		break;

	/* Undefined log type - just exit          */
	default:
		return 0;
	}

	/* Log only if the mode mask allows */
	if (type == LTT_LITE_EV_ANDROID_RADIO) {
		if (!(ltt_lite_mode & LTT_LITE_MODE_ANDROID_RADIO))
			return 0;
	} else {
		if (!(ltt_lite_mode & LTT_LITE_MODE_ANDROID))
			return 0;
	}

	/* Setup local buffer parameters for copy from userspace vectors */
	record = (struct ltt_lite_android_event *) ltt_lite_small_buffer;
	recptr = (char *) &(record->data);
	/* offsetof(ltt_lite_android_event, data)? */
	reclen = sizeof(record_header_t) + 4;
	record->pid = current->pid;   /* Record the PID for the event */

	/* Get the vector data from userspace */
	for (vecs = 0; vecs < nr_segs; vecs++)	{
		/* Discard the event if it exceeds the static buffer size */
		if ((reclen + iov->iov_len) > LTT_LITE_SMALL_BUFFER_SIZE)
			return 0;

		/* Discard the event if all data was not copied */
		if (copy_from_user(recptr, iov->iov_base, iov->iov_len))
			return 0;

		/* Advance the pointer in the record buffer  */
		recptr += iov->iov_len;
		/* Increment the size of the record buffer payload portion */
		reclen += iov->iov_len;
		/* Point to the next vector (userspace data) */
		iov++;
	}

	/* Write populated record buffer into main LTT-lite log buffer */
	commit_log(record, reclen, type);
	return android_logging_mode;
}

/*
 * This interface is for logging the printk message stream,
 * for graphical interpretation by the Taskview postprocessing tool.
 */
void ltt_lite_log_printk(char *string, int size)
{
	if (!(ltt_lite_is_enabled | early_enabled_mode) ||
			!(ltt_lite_mode & LTT_LITE_MODE_PRINTK))
		return;

	if ((sizeof(record_header_t) + size) <
					LTT_LITE_SMALL_BUFFER_SIZE) {
		memcpy(ltt_lite_small_buffer + sizeof(record_header_t),
						string, size);
		commit_log(ltt_lite_small_buffer, sizeof(record_header_t) +
						size,
						LTT_LITE_EV_PRINTK);
	} else {
		memcpy(ltt_lite_small_buffer + sizeof(record_header_t),
					string, LTT_LITE_SMALL_BUFFER_SIZE);
		/* Truncate/terminate string */
		ltt_lite_small_buffer[LTT_LITE_SMALL_BUFFER_SIZE - 1] = 0;
		commit_log(ltt_lite_small_buffer,
						LTT_LITE_SMALL_BUFFER_SIZE,
						LTT_LITE_EV_PRINTK);
	}
}
#endif
