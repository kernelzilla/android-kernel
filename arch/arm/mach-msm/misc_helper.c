/* arch/arm/mach-msm/misc_helper.c
 *
 * Driver to perform different kernel calls
 *
 */
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <mach/vreg.h>
#include <linux/clk.h>

#include "proc_comm.h"
#include "board-mot.h"
#include "smd_private.h"

#define DEBUG
/* #undef DEBUG */
#ifdef DEBUG
#define D(x...) printk(x)
#else
#define D(x...) do {} while (0)
#endif

#define MH_RESP_NOT_REQUIRED	0x00000000
#define MH_RESP_REQUIRED	0x00000001
#define MH_RESP_READY		0x00000002
#define MH_RESP_FMT_UINT	0x01000000
#define MH_RESP_FMT_IINT	0x02000000
#define MH_RESP_FMT_UCHR	0x04000000
#define MH_RESP_FMT_BUFF	0x08000000

extern int is_secure_hw (void);

struct misc_helper_inf {
    unsigned int response;
    union {
	unsigned int	uint;
	int	    	iint;
	unsigned char	uchr;
	char		buffer[1024];
    }		 data;
}	misc_helper_info;


static ssize_t misc_helper_read(struct file *fp, char __user *buf, size_t count, loff_t *pos)
{
	char buffer[1024];
	count = 0;
	if (misc_helper_info.response & MH_RESP_REQUIRED) {
	  if (misc_helper_info.response & MH_RESP_READY) {
		unsigned int fmt = misc_helper_info.response & 0xFF000000;
		switch (fmt) {
		 case MH_RESP_FMT_UINT :
			count = snprintf (buffer, 1023, "%u\n", misc_helper_info.data.uint); break;
		 case MH_RESP_FMT_IINT :
			count = snprintf (buffer, 1023, "%d\n", misc_helper_info.data.iint); break;
		 case MH_RESP_FMT_UCHR :
			count = snprintf (buffer, 1023, "0x%02x\n", misc_helper_info.data.uchr); break;
		 case MH_RESP_FMT_BUFF :
			count = snprintf (buffer, 1023, "%s\n", misc_helper_info.data.buffer); break;
		}
		misc_helper_info.response = 0;
	  } else {
		count = sprintf (buffer, "Response is expected soon\n");
	  }
	  if (copy_to_user (buf, buffer, count))
		count = 0;
	}
	return count;
}

#include <linux/ctype.h>

static int misc_helper_args(char **args, char *cmd, int len)
{
	int arg_num=0;
	char *s, *e, *p=cmd;
	e = p + len;
	//D(KERN_ERR "start: %p, end: %p\n", p, e);
	while (p < e) {
		while ((p < e) && (*p == ' '))	// trim white spaces
			p++;
		s = p;				// save argument start
		while ((p < e) && ! isspace(*p))	// find end
			p++;
		if (p == s) {
			//D(KERN_ERR "leaving at: %p\n", p);
			break;
		}
		*p = 0;			// put '\0'
		args[arg_num] = s;	// save argument to return
		D(KERN_ERR "arg[%d] is '%s'\n", arg_num, args[arg_num]);
		arg_num++;
		p++;			// go to the next character
	}
	D(KERN_ERR "args # is %d\n", arg_num);
	return arg_num;
}

void pretty_hex_dump (const char *title, unsigned char *buf, int len)
{
	static char buffer[128];
	unsigned char *ptr = NULL;
	int n, l, w, total = 0;
	printk(KERN_INFO "%s hex dump from addr %08x, len %d\n", title, (unsigned int)buf, len);
	for (n=0, l=0; n < len; n++) {
	    if (!(n%16)) {
		total = 0;
		ptr = buffer;
	        w = snprintf (ptr, 128-total, "%04x: %02x ", l*16, *(buf+n));
	    } else
	        w = snprintf (ptr, 128-total, "%02x ", *(buf+n));
	    total += w;
	    ptr += w;
	    if ((n%16) == 15) {
		printk(KERN_INFO "%s\n", buffer);
		l++;
	    }
	}
	if (total) printk(KERN_INFO "%s\n", buffer);
}
EXPORT_SYMBOL (pretty_hex_dump);

extern int gpio_get_value (unsigned gpio);
extern int gpio_set_value (unsigned gpio, int on);

#define MISC_HELPER_DEFS  \
    KEYWORD("help",            0, CMD_HELP)	\
    KEYWORD("gpio_get",        1, CMD_GPIO_GET)	\
    KEYWORD("gpio_set",        2, CMD_GPIO_SET)	\
    KEYWORD("vreg",            2, CMD_VREG) \
    KEYWORD("clk",             2, CMD_CLK) \
    KEYWORD("reset_modem",     0, CMD_RESET)	\
    KEYWORD("chip_power_down", 0, CMD_CHIP_PWRDN)	\
    KEYWORD("battery",         0, CMD_BATTERY)	\
    KEYWORD("charging",        0, CMD_CHARGING)	\
    KEYWORD("uuid_high",       0, CMD_UUID_HIGH) \
    KEYWORD("uuid_low",        0, CMD_UUID_LOW) \
    KEYWORD("nvread",          1, CMD_NVREAD)	\
    KEYWORD("nvwrite",         2, CMD_NVWRITE)  \
    KEYWORD("bp_ver",          0, CMD_BP_VER) \
    KEYWORD("ap_flash",        0, CMD_AP_FLASH) \
    KEYWORD("trusted",         0, CMD_TRUSTED_BOOT) \
    KEYWORD("secure",          0, CMD_SECURE) \
    KEYWORD("fb_set",          0, CMD_FB_SET) \
    KEYWORD("fb_get",          0, CMD_FB_GET)
/*
//    KEYWORD("nv_read",         1, CMD_NVR)  \
//    KEYWORD("nv_write",        2, CMD_NVW) \
//    KEYWORD("",  0, CMD_)
*/
#define KEYWORD(a,b,c)	c,
enum {
	MISC_HELPER_DEFS
	CMD_MAX
};
#undef KEYWORD

#define KEYWORD(a,b,c)	a,
static char *misc_helper_cmd_name[CMD_MAX]={MISC_HELPER_DEFS};
#undef KEYWORD

#define KEYWORD(a,b,c)	b,
static int misc_helper_cmd_argc[CMD_MAX]={MISC_HELPER_DEFS};
#undef KEYWORD

static int misc_helper_get_cmd(char *cmd, int argc)
{
	int i;
    argc--; // don't count the command itself
	for (i=CMD_MAX-1; i >= 0; i--)
		if (strlen (cmd) == strlen (misc_helper_cmd_name[i]) &&
		    ! strncmp (cmd, misc_helper_cmd_name[i], strlen (misc_helper_cmd_name[i]))) {
			D(KERN_ERR "cmd is %s\n", cmd);
			if (argc < misc_helper_cmd_argc[i]) {
				D(KERN_ERR "missing arguments %d(%d) for cmd %s\n", argc, misc_helper_cmd_argc[i], cmd);
				i = CMD_MAX;
			}
			break;
		}
	return i;
}


static void misc_helper_usage(char *args[], int argc)
{
	D(KERN_ERR "usage for cmd %s (argc=%d)\n", args[0], argc);
	if (1 == argc) {
		volatile int cp=0, to=1023;
		int i, n;
		n = snprintf (misc_helper_info.data.buffer, to, "supported commands:\n");
		cp += n;
		to -= n;
		for (i=0;i<CMD_MAX;i++) {
			n = snprintf (misc_helper_info.data.buffer+cp, to, "%s\n", misc_helper_cmd_name[i]);
			cp += n;
			to -= n;
			D(KERN_ERR "%s i=%d, n=%d, cp=%d, to=%d\n", misc_helper_cmd_name[i], i, n, cp, to);
		}
		n = snprintf (misc_helper_info.data.buffer+cp, to, "try 'help cmd'");
		cp += n;
		to -= n;
		D(KERN_ERR "written %d to: %s\n", 1023-to, misc_helper_info.data.buffer);
	} else {
		switch (misc_helper_get_cmd (args[1], 999999))
		 {
		 case CMD_FB_GET :
			snprintf (misc_helper_info.data.buffer, 1023,
				"reads factory byte\nusage: %s", args[1]);
		 case CMD_FB_SET :
			snprintf (misc_helper_info.data.buffer, 1023,
				"writes factory byte\nusage: %s", args[1]);
		 case CMD_BP_VER :
			snprintf (misc_helper_info.data.buffer, 1023,
				"reads BP build version\nusage: %s", args[1]);
				break;
		 case CMD_AP_FLASH :
			snprintf (misc_helper_info.data.buffer, 1023,
				"sets phone into AP flash mode\nusage: %s", args[1]);
				break;

		 case CMD_UUID_HIGH :
			snprintf (misc_helper_info.data.buffer, 1023,
				"reads UUID high\nusage: %s", args[1]);
				break;
		 case CMD_UUID_LOW :
			snprintf (misc_helper_info.data.buffer, 1023,
				"reads UUID low\nusage: %s", args[1]);
				break;
		 case CMD_TRUSTED_BOOT :
			snprintf (misc_helper_info.data.buffer, 1023,
				"shows trusted boot status\nusage: %s", args[1]);
				break;
		 case CMD_SECURE :
			snprintf (misc_helper_info.data.buffer, 1023,
				"shows security status\nusage: %s", args[1]);
				break;
		 case CMD_GPIO_GET :
			snprintf (misc_helper_info.data.buffer, 1023,
				"shows GPIO's state (AP only)\nusage: %s gpio\n  gpio - gpio number", args[1]);
				break;
		 case CMD_GPIO_SET :
			snprintf (misc_helper_info.data.buffer, 1023,
				"sets GPIO's state (AP only)\nusage: %s gpio state\n  gpio - gpio number\n  state - 0/1", args[1]);
				break;
         case CMD_VREG :
            snprintf (misc_helper_info.data.buffer, 1023,
                "set VREG state\nusage: %s vreg_name mV\n  vreg_name - see kernel/arch/arm/mach-msm/vreg.c\n  mV - millivolts (0 for off)", args[1]);
                break;
         case CMD_CLK :
            snprintf (misc_helper_info.data.buffer, 1023,
                "set CLK state\nusage: %s clk_name rate\n  clk_name - see kernel/arch/arm/mach-msm/devices.c\n  rate - clk rate in Hz (0 for off)", args[1]);
                break;
		 case CMD_BATTERY :
			snprintf (misc_helper_info.data.buffer, 1023,
				"shows battery's charge level\nusage: %s", args[1]);
				break;
		 case CMD_CHARGING :
			snprintf (misc_helper_info.data.buffer, 1023,
				"shows if charging is occuring\nusage: %s", args[1]);
				break;
		 case CMD_RESET :
			snprintf (misc_helper_info.data.buffer, 1023,
				"resets BP (ungracefully)\nusage: %s", args[1]);
				break;
		 case CMD_CHIP_PWRDN :
			snprintf (misc_helper_info.data.buffer, 1023,
				"powers down whole chip (ungracefully)\nusage: %s", args[1]);
				break;
		 case CMD_NVREAD :
			snprintf (misc_helper_info.data.buffer, 1023,
				"reads NV item\nusage: %s nvitem [nmelem]\nnvitem - NV item number\nnvelem - item's element (unsigned int)", args[1]);
				break;
		 case CMD_NVWRITE :
			snprintf (misc_helper_info.data.buffer, 1023,
				"writes NV item\nusage: %s nvitem [nmelem]\nnvitem - NV item number\nnvelem - item's element (unsigned int)", args[1]);
				break;
#if 0
		 case CMD_NVR :
			snprintf (misc_helper_info.data.buffer, 1023,
				"reads NV item\nusage: %s nvitem [nmelem]\nnvitem - NV item number\nnvelem - item's element (unsigned int)", args[1]);
				break;
		 case CMD_NVW :
			snprintf (misc_helper_info.data.buffer, 1023,
				"writes NV item\nusage: %s nvitem [nmelem]\nnvitem - NV item number\nnvelem - item's element (unsigned int)", args[1]);
				break;
#endif
		 case CMD_HELP :
		 default : sprintf (misc_helper_info.data.buffer, "help is not available for command: '%s'", args[1]);
		 }
	}
}

char *socinfo_get_baseband_id(void);

static ssize_t misc_helper_write(struct file *fp, const char __user *buf, size_t count, loff_t *pos)
{
	unsigned char cmd[64];
	int len;
	int i, r;
	unsigned int id, val;
	unsigned char *uchar;
	char *soc_baseband_id, *args[10]={0};

	static char bp_version[32]={0};
	static int  bp_taken=0;
	static smem_mot_vendor1_type *vendor1;

	if (count < 1)
		return 0;

	len = count > 63 ? 63 : count;

	if (copy_from_user(cmd, buf, len))
		return -EFAULT;

	cmd[len] = 0;

	/* lazy */
	if (cmd[len-1] == '\n') {
		cmd[len-1] = 0;
		len--;
	}

	if ((r = misc_helper_args (args, cmd, len)) > 0) {
		for (i=0; i < r; i++)
			D(KERN_ERR "ARG[%d] %s\n", i, args[i]);
	}

    if (r == 0)
        return count;

	switch (misc_helper_get_cmd (args[0], r))
	 {
	 case CMD_BP_VER :
		misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
		if (! bp_taken) {
			soc_baseband_id = socinfo_get_baseband_id();
			if (soc_baseband_id) {
				strncpy (bp_version, soc_baseband_id, sizeof (bp_version)-1);
				bp_version[31] = 0;
			}else
				strcpy (bp_version, "undefined");
			bp_taken = 1;
		}
		D("%s(): %s\n", misc_helper_cmd_name[CMD_BP_VER], bp_version);
		sprintf (misc_helper_info.data.buffer, "%s", bp_version);
			break;

	 case CMD_AP_FLASH :
		if(!0) {
			misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
			id = PROCCOMM_MODEM_SET_AP_FLASH_REASON;
			val = 0;
			r = meta_proc(id, &val);
			D("%s()=%08x,%08x rc=%d\n", misc_helper_cmd_name[CMD_AP_FLASH], id, val, r);
			sprintf (misc_helper_info.data.buffer, "%u %u", id, val);
		} else
			sprintf (misc_helper_info.data.buffer, "Permissions denied");
			break;

	 case CMD_UUID_HIGH :
		misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
		id = 0;
		val = 0;
		r = msm_proc_comm (PCOM_GET_UUID_HIGH, &id, &val);
		D("%s()=%08x,%08x rc=%d\n", misc_helper_cmd_name[CMD_UUID_HIGH], id, val, r);
		uchar = (unsigned char *)&id;
		sprintf (misc_helper_info.data.buffer, "%02x %02x %02x %02x ", *uchar, *uchar+1, *uchar+2, *uchar+3);
		uchar = (unsigned char *)&val;
		sprintf (misc_helper_info.data.buffer+strlen(misc_helper_info.data.buffer),
			 "%02x %02x %02x %02x", *uchar, *uchar+1, *uchar+2, *uchar+3);
			break;

	 case CMD_UUID_LOW :
		misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
		id = 0;
		val = 0;
		r = msm_proc_comm (PCOM_GET_UUID_LOW, &id, &val);
		D("%s()=%08x,%08x rc=%d\n", misc_helper_cmd_name[CMD_UUID_LOW], id, val, r);
		uchar = (unsigned char *)&id;
		sprintf (misc_helper_info.data.buffer, "%02x %02x %02x %02x ", *uchar, *uchar+1, *uchar+2, *uchar+3);
		uchar = (unsigned char *)&val;
		sprintf (misc_helper_info.data.buffer+strlen(misc_helper_info.data.buffer),
			 "%02x %02x %02x %02x", *uchar, *uchar+1, *uchar+2, *uchar+3);
			break;

	 case CMD_HELP :
		if (!is_secure_hw()) {
			misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
			misc_helper_usage (args, r);
		} else
			sprintf (misc_helper_info.data.buffer, "Permissions denied");
			break;

	 case CMD_GPIO_GET :
		misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
		id = (unsigned)simple_strtoul (args[1], NULL, 10);
		val = gpio_get_value (id);
		D("%s(%u)=%u\n", misc_helper_cmd_name[CMD_GPIO_GET], id, val);
		sprintf (misc_helper_info.data.buffer, "%u", val);
			break;

	 case CMD_BATTERY :
		misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
		id = 0;
		val = 0;
		r = msm_proc_comm (PCOM_GET_BATT_LEVEL, &id, &val);
		D("%s()=%u,%u rc=%d\n", misc_helper_cmd_name[CMD_BATTERY], id, val, r);
		sprintf (misc_helper_info.data.buffer, "%u %u", id, val);
			break;

	 case CMD_CHARGING :
		misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
		id = 0;
		val = 0;
		r = msm_proc_comm (PCOM_CHG_IS_CHARGING, &id, &val);
		D("%s()=%u,%u rc=%d\n", misc_helper_cmd_name[CMD_CHARGING], id, val, r);
		sprintf (misc_helper_info.data.buffer, "%u %u", id, val);
			break;

	 case CMD_RESET :
		if(!0) { //if (!is_secure_hw()) {
			misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
			id = 0;
			val = 0;
			r = msm_proc_comm (PCOM_RESET_MODEM, &id, &val);
			D("%s()=%u,%u rc=%d\n", misc_helper_cmd_name[CMD_RESET], id, val, r);
			sprintf (misc_helper_info.data.buffer, "%u %u", id, val);
		} else
			sprintf (misc_helper_info.data.buffer, "Permissions denied");
			break;

	 case CMD_CHIP_PWRDN :
		if(!0) { //if (!is_secure_hw()) {
			misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
			id = 0;
			val = 0;
			r = msm_proc_comm (PCOM_POWER_DOWN, &id, &val);
			D("%s()=%u,%u rc=%d\n", misc_helper_cmd_name[CMD_CHIP_PWRDN], id, val, r);
			sprintf (misc_helper_info.data.buffer, "%u %u", id, val);
		} else
			sprintf (misc_helper_info.data.buffer, "Permissions denied");
			break;

	 case CMD_NVREAD :
		if (!0) { //if (!is_secure_hw()) {
			misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
			id = (unsigned)simple_strtoul (args[1], NULL, 10);
			id = (id << 8) | PROCCOMM_NV_READ;
			val = 0;
			r = msm_proc_comm (PCOM_CUSTOMER_CMD3, &id, &val);
			D("%s(0x%08x)=%u rc=%d\n", misc_helper_cmd_name[CMD_NVREAD], id, val, r);
			sprintf (misc_helper_info.data.buffer, "%u %u", id, val);
		} else
			sprintf (misc_helper_info.data.buffer, "Permissions denied");
			break;

	 case CMD_NVWRITE :
		if (!0) { //if (!is_secure_hw()) {
			misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
			id = (unsigned)simple_strtoul (args[1], NULL, 10);
			id = (id << 8) | PROCCOMM_NV_WRITE;
			val = (unsigned)simple_strtoul (args[2], NULL, 10);
			r = msm_proc_comm (PCOM_CUSTOMER_CMD3, &id, &val);
			D("%s(0x%08x, %u) rc=%d\n", misc_helper_cmd_name[CMD_NVWRITE], id, val, r);
			sprintf (misc_helper_info.data.buffer, "%d", r);
		} else
			sprintf (misc_helper_info.data.buffer, "Permissions denied");
			break;

#if 0
	 case CMD_NVR :
		misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
		id = (unsigned)simple_strtoul (args[1], NULL, 10);
		val = 0; 
		r = msm_proc_comm (PCOM_NV_READ, &id, &val);
		D("%s(%u)=%u rc=%d\n", misc_helper_cmd_name[CMD_NVR], id, val, r);
		sprintf (misc_helper_info.data.buffer, "%u %u", id, val);
			break;

	 case CMD_NVW :
		misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
		id = (unsigned)simple_strtoul (args[1], NULL, 10);
		val = (unsigned)simple_strtoul (args[2], NULL, 10);
		r = msm_proc_comm (PCOM_NV_WRITE, &id, &val);
		D("%s(%u, %u) rc=%d\n", misc_helper_cmd_name[CMD_NVW], id, val, r);
		sprintf (misc_helper_info.data.buffer, "%d", r);
			break;
#endif

	 case CMD_GPIO_SET :
		if (!is_secure_hw()) {
			misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
			id = (unsigned)simple_strtoul (args[1], NULL, 10);
			val = (unsigned)simple_strtoul (args[2], NULL, 10);
			r = gpio_set_value (id, val);
			val = gpio_get_value (id);
			D("%s(%u, %u) rc=%d\n", misc_helper_cmd_name[CMD_GPIO_SET], id, val, r);
			sprintf (misc_helper_info.data.buffer, "%u", val);
		} else
			sprintf (misc_helper_info.data.buffer, "Permissions denied");
			break;

     case CMD_VREG :
	if (!is_secure_hw()) {
           misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
           {
            unsigned int mv = (unsigned)simple_strtoul (args[2], NULL, 10);
            int rc;
            struct vreg *vreg;
            vreg = vreg_get(0,args[1]);
            if (IS_ERR(vreg)) {
                sprintf(misc_helper_info.data.buffer, "* bad name (%s)", args[1]);
                break;
            }
            if (mv > 0) {
                if ((rc = vreg_set_level(vreg, mv))) {
                    sprintf(misc_helper_info.data.buffer, "* failed to set level (%d mV) [%d]", mv, rc);
                    break;
                }
                if ((rc = vreg_enable(vreg))) {
                    sprintf(misc_helper_info.data.buffer, "* failed to enable [%d]", rc);
                    break;
                }
            }
            else {
                if ((rc = vreg_disable(vreg))) {
                    sprintf(misc_helper_info.data.buffer, "* failed to disable [%d]", rc);
                    break;
                }
            }
            D("%s(%s,%d) rc=%d\n", misc_helper_cmd_name[CMD_VREG],args[1],mv,rc);
           }
           sprintf(misc_helper_info.data.buffer, "OK");
	} else
		sprintf (misc_helper_info.data.buffer, "Permissions denied");
        		break;

     case CMD_CLK :
	if (!is_secure_hw()) {
	   misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
	   {
            unsigned int rate = (unsigned)simple_strtoul (args[2], NULL, 10);
            int rc = 0;
            struct clk *clk;
            clk = clk_get(NULL, args[1]);
            if (IS_ERR(clk)) {
                sprintf(misc_helper_info.data.buffer, "* bad name (%s)", args[1]);
                break;
            }
            if (rate > 0) {
                if ((rc = clk_enable(clk))) {
                    sprintf(misc_helper_info.data.buffer, "* failed to enable clk [%d]", rc);
                    break;
                }
                if ((rc = clk_set_rate(clk,rate))) {
                    sprintf(misc_helper_info.data.buffer, "* failed to set rate [%d]", rc);
                    break;
                }
            }
            else {
                clk_disable(clk);
            }
            D("%s(%s,%d) rc=%d\n", misc_helper_cmd_name[CMD_CLK],args[1],rate,rc);
           }
           sprintf(misc_helper_info.data.buffer, "OK");
	} else
		sprintf (misc_helper_info.data.buffer, "Permissions denied");
        		break;

	 case CMD_FB_SET :
		if (!is_secure_hw()) {
			misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
			id = (1 << 8) | PROCCOMM_FACTORY_BYTE;
			val = (unsigned)simple_strtoul (args[1], NULL, 10);
			r = msm_proc_comm (PCOM_CUSTOMER_CMD3, &id, &val);
			D("%s(%u)=%u\n", misc_helper_cmd_name[CMD_FB_SET], id, val);
			sprintf (misc_helper_info.data.buffer, "%u", val);
		} else
			sprintf (misc_helper_info.data.buffer, "Permissions denied");
			break;

	 case CMD_TRUSTED_BOOT :
		if (! vendor1) {
			vendor1 = smem_alloc(SMEM_ID_VENDOR1, sizeof(smem_mot_vendor1_type));
		}
		if (vendor1) {
			r = 0;
			val = (unsigned int)vendor1->trusted_boot;

		} else {
			r = -1;
			val = -1;
		}
		misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
		D("%s(is_boot_trusted)=%u, rc=%d\n", misc_helper_cmd_name[CMD_TRUSTED_BOOT], val, r);
		sprintf (misc_helper_info.data.buffer, "%u", val);
			break;


	 case CMD_SECURE :
		r = 0;
		val = (unsigned int)is_secure_hw();
		misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
		D("%s(is_security_on)=%u, rc=%d\n", misc_helper_cmd_name[CMD_SECURE], val, r);
		sprintf (misc_helper_info.data.buffer, "%u", val);
			break;


	 case CMD_FB_GET :
		if (! vendor1) {
			vendor1 = smem_alloc(SMEM_ID_VENDOR1, sizeof(smem_mot_vendor1_type));
		}
		if (vendor1) {
			r = 0;
			val = (unsigned int)vendor1->fact_byte;

		} else {
			r = -1;
			val = -1;
		}
		misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
		D("%s(10024)=%u, rc=%d\n", misc_helper_cmd_name[CMD_FB_GET], val, r);
		sprintf (misc_helper_info.data.buffer, "%u", val);
			break;

	 case CMD_MAX :
		misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF;
		sprintf (misc_helper_info.data.buffer, "missing parameters for command: %s", args[0]);
			break;
	 default :
		misc_helper_info.response |= MH_RESP_REQUIRED | MH_RESP_READY | MH_RESP_FMT_BUFF; 
		sprintf (misc_helper_info.data.buffer, "command: %s is not supported", cmd);
	}
	return count;
}

static int misc_helper_open(struct inode *ip, struct file *fp)
{
	return 0;
}

static int misc_helper_release(struct inode *ip, struct file *fp)
{
	return 0;
}

static struct file_operations misc_helper_fops = {
	.owner = THIS_MODULE,
	.read = misc_helper_read,
	.write = misc_helper_write,
	.open = misc_helper_open,
	.release = misc_helper_release,
};

static struct miscdevice misc_helper_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "misc_helper",
	.fops = &misc_helper_fops,
};

static int __init misc_helper_init(void)
{
	memset (&misc_helper_info, 0, sizeof(misc_helper_info));
	return misc_register(&misc_helper_dev);
}

module_init(misc_helper_init);

MODULE_DESCRIPTION("Kernel based TCMD agent");
MODULE_LICENSE("GPL v2");
