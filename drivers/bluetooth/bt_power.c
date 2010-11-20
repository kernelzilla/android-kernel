#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <mach/vreg.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <linux/delay.h>
#include <mach/msm_serial_hs.h>


//////////////////////////////////////////////////////////////////////
static struct proc_dir_entry *proc_entry;
static int power_is_on;

#if defined(CONFIG_KERNEL_MOTOROLA)

#include <board-mot.h>

#define INTERNAL_BT_VREG        BT_REG_ON_SIGNAL
#define BT_RESET                BT_RESET_N_SIGNAL
#define BT_WAKE                 BT_EXT_WAKE_SIGNAL
#define BT_HOST_WAKE            BT_HOST_WAKE_SIGNAL

#else /* defined(CONFIG_KERNEL_MOTOROLA) */

#define WLAN_REG_ON_SIGNAL 	87
#define INTERNAL_BT_VREG        92
#define BT_RESET                93
#define BT_WAKE                 91
#define BT_HOST_WAKE            90

#endif /* defined(CONFIG_KERNEL_MOTOROLA) */

#if defined(CONFIG_MACH_PITTSBURGH)
#define vreg_bt "gp5"
#else
#define vreg_bt "gp3"
#endif /* defined(CONFIG_MACH_PITTSBURGH) */
//////////////////////////////////////////////////////////////////////
static int bt_power_read_proc(char *buffer, char **start, off_t offset, int size, int *eof, void *data)
{
  char *str;
  int r;
  int len;
  
  r = gpio_get_value(INTERNAL_BT_VREG);  // internal BT VREG
  
  if(r)
    {str = "1 (Bluetooth power is ON)\n";}
  else
    {str = "0 (Bluetooth power is OFF)\n";}
  
  len = strlen(str);
  if (size < len) {return -EINVAL;}
  if (offset != 0){return 0;}
  
  strcpy(buffer, str);
  
  *eof = 1;
  return len;
}

//////////////////////////////////////////////////////////////////////
void xstrcat(char *d, char *s)
{
  while(*d){d++;}
  while(*s){*d++ = *s++;}
  *d=0;
}

//////////////////////////////////////////////////////////////////////
static int bt_stat_read_proc(char *buffer, char **start, off_t offset, int size, int *eof, void *data)
{
  static unsigned int op;
  unsigned int cp;
  char str[64*13];
  char line[64];
  int len=0;
  str[0]=0;

  cp=0;


//////////////////////////////////////////////////////////////////////

  cp |= gpio_get_value(BT_RESET); cp <<= 1;
  cp |= gpio_get_value(INTERNAL_BT_VREG); cp <<= 1;
  cp |= gpio_get_value(WLAN_REG_ON_SIGNAL); cp <<= 1;
  cp |= gpio_get_value(43); cp <<= 1;
  cp |= gpio_get_value(44); cp <<= 1;
  cp |= gpio_get_value(45); cp <<= 1;
  cp |= gpio_get_value(46); cp <<= 1;
  cp |= gpio_get_value(68); cp <<= 1;
  cp |= gpio_get_value(69); cp <<= 1;
  cp |= gpio_get_value(70); cp <<= 1;
  cp |= gpio_get_value(71); cp <<= 1;
  cp |= gpio_get_value(BT_WAKE); cp <<= 1;
  cp |= gpio_get_value(BT_HOST_WAKE);

//////////////////////////////////////////////////////////////////////

  if(cp != op)
    {
      op = cp;

      sprintf(line,"%d",   gpio_get_value(BT_RESET)); strcat(str,line);
      sprintf(line,"%d",   gpio_get_value(INTERNAL_BT_VREG)); strcat(str,line);
      sprintf(line,"%d ",  gpio_get_value(WLAN_REG_ON_SIGNAL)); strcat(str,line);
      
      sprintf(line,"%d",   gpio_get_value(43)); strcat(str,line);
      sprintf(line,"%d",   gpio_get_value(44)); strcat(str,line);
      sprintf(line,"%d",   gpio_get_value(45)); strcat(str,line);
      sprintf(line,"%d ",  gpio_get_value(46)); strcat(str,line);
      
      sprintf(line,"%d",   gpio_get_value(68)); strcat(str,line);
      sprintf(line,"%d",   gpio_get_value(69)); strcat(str,line);
      sprintf(line,"%d",   gpio_get_value(70)); strcat(str,line);
      sprintf(line,"%d ",  gpio_get_value(71)); strcat(str,line);
      
      sprintf(line,"%d",   gpio_get_value(BT_WAKE)); strcat(str,line);
      sprintf(line,"%d\n", gpio_get_value(BT_HOST_WAKE)); strcat(str,line);

      len = strlen(str);
      if (size < len) {return -EINVAL;}
    }

  if (offset != 0){return 0;}
  
  strcpy(buffer, str);
  
  *eof = 1;
  return len;
}

//////////////////////////////////////////////////////////////////////
static int bt_status_read_proc(char *buffer, char **start, off_t offset, int size, int *eof, void *data)
{
  char str[64*13];
  char line[64];
  int len;

  str[0]=0;

  sprintf(line,"%d = BT_RESET\n",   gpio_get_value(BT_RESET)); strcat(str,line);
  sprintf(line,"%d = BT_VREG\n",    gpio_get_value(INTERNAL_BT_VREG)); strcat(str,line);
  sprintf(line,"%d = WLAN_VREG\n\n",gpio_get_value(WLAN_REG_ON_SIGNAL)); strcat(str,line);

  sprintf(line,"%d = RFR\n",        gpio_get_value(43)); strcat(str,line);
  sprintf(line,"%d = CTS\n",        gpio_get_value(44)); strcat(str,line);
  sprintf(line,"%d = Rx\n",         gpio_get_value(45)); strcat(str,line);
  sprintf(line,"%d = Tx\n\n",       gpio_get_value(46)); strcat(str,line);

  sprintf(line,"%d = PCM_DOUT\n",   gpio_get_value(68)); strcat(str,line);
  sprintf(line,"%d = PCM_DIN\n",    gpio_get_value(69)); strcat(str,line);
  sprintf(line,"%d = PCM_SYNC\n",   gpio_get_value(70)); strcat(str,line);
  sprintf(line,"%d = PCM_CLK\n\n",  gpio_get_value(71)); strcat(str,line);

  sprintf(line,"%d = WAKE\n",       gpio_get_value(BT_WAKE)); strcat(str,line);
  sprintf(line,"%d = HOST_WAKE\n\n",gpio_get_value(BT_HOST_WAKE)); strcat(str,line);

  len = strlen(str);
  if (size < len) {return -EINVAL;}
  if (offset != 0){return 0;}
  
  strcpy(buffer, str);
  
  *eof = 1;
  return len;
}

//////////////////////////////////////////////////////////////////////
static int bt_irq_read_proc(char *buffer, char **start, off_t offset, int size, int *eof, void *data)
{
  char line[1024];
  char buf[32];
  unsigned int irq[65];
  extern int interrupt_ack_stats[];
  int len;
  
  int i=65;
  do
    {
      i--;
      irq[i]=interrupt_ack_stats[i];  // note: interrupt_ack_stats[] is in kernel/arch/arm/mach-msm/irq.c
      interrupt_ack_stats[i]=0;
    }
  while(i);
  
  i=0; line[0]=0;
  do
    {
      if(irq[i])
       {
         sprintf(buf,"%02d->%u\n",i,irq[i]);
         strncat(line,buf,1023);
       }
      i++;
    }
  while(i <= 64);
  strncat(line,"\n",1023);

  len = strlen(line);
  if (size < len) {return -EINVAL;}
  if (offset != 0){return 0;}
  
  strcpy(buffer, line);
  
  *eof = 1;
  return len;
}

//////////////////////////////////////////////////////////////////////

void hsuart_power(int on)
{
  // this is incomplete; don't use.
  return;
#ifndef CONFIG_MACH_MOT
  struct uart_port *ups;

  if (on) 
    {
      msm_hs_request_clock_on(ups);
      msm_hs_set_mctrl_locked(ups, TIOCM_RTS);
    } 
  else 
    {
      msm_hs_set_mctrl_locked(ups, 0);
      msm_hs_request_clock_off(ups);
    }
#endif
}

//////////////////////////////////////////////////////////////////////
static char proc_buf[1];
extern int A2DP_prioritization_trigger;
// write '1' to turn on power, '0' to turn it off
// write 'w' to set BT WAKE to 0,  'W' to set BT WAKE to 1
// write 'a' to trigger A2DP mode transition to send broadcom VSC for BT prioritization
int bt_power_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
  struct vreg *vreg;
  int rc;

  if(copy_from_user(proc_buf, buffer, 1)) {return(-EFAULT);}

  if('a' == *proc_buf)
    {
      printk(KERN_INFO "BLUETOOTH: A2DP VSC for BT prioritization triggered (was %d)\n",
             A2DP_prioritization_trigger);
      A2DP_prioritization_trigger = 1;
#if defined(CONFIG_MACH_MOT)
	return 0;
#else
	return 1;
#endif
    }

  if('w' == *proc_buf)
    {
      gpio_set_value(BT_WAKE, 0);
      printk(KERN_INFO "BLUETOOTH: BT_WAKE: low\n");
#if defined(CONFIG_MACH_MOT)
	return 0;
#else
	return 1;
#endif
    }
  
  if('W' == *proc_buf)
    {
      gpio_set_value(BT_WAKE, 1);
      printk(KERN_INFO "BLUETOOTH: BT_WAKE: high\n");
#if defined(CONFIG_MACH_MOT)
	return 0;
#else
	return 1;
#endif
    }
  
  if('1' == *proc_buf)
    {
      if(!power_is_on)
	{

#ifndef CONFIG_MACH_MOT
if
(
	  !machine_is_calgary() &&
	  !machine_is_msm7627_pittsburgh()
)
{
#endif
	  // Turn on VREG_BT_PA
	  vreg = vreg_get(0, vreg_bt);
	  if ((rc = vreg_enable(vreg)))
	    {printk(KERN_ERR "BLUETOOTH: %s: vreg(%s) enable failed (%d)\n", __func__,vreg_bt, rc);}
#ifndef CONFIG_MACH_MOT
}
#endif
	  
	  msleep_interruptible(100);
	  
	  gpio_request(INTERNAL_BT_VREG, "bt_reg_on"); 	// turn on internal BT VREG
	  gpio_direction_output(INTERNAL_BT_VREG, 1);
	  
	  msleep_interruptible(100);	// BCM4325 powerup requirement
	  
	  gpio_request(BT_RESET, "bt_reset_n"); // take BT out of reset
	  gpio_direction_output(BT_RESET, 1);
	  
	  power_is_on = 1;
	  printk(KERN_INFO "BLUETOOTH: bt_power: ON\n");
	}
    }
  else
    {
      if(power_is_on)
	{
	  gpio_set_value (BT_RESET, 0);	// put BT into reset
	  gpio_set_value (INTERNAL_BT_VREG, 0);	// turn off internal BT VREG
	  
	  if(!gpio_get_value(WLAN_REG_ON_SIGNAL))
	    {
#ifndef CONFIG_MACH_MOT
if
(
	  !machine_is_calgary() &&
	  !machine_is_msm7627_pittsburgh()
)
{
#endif
	      // Turn off VREG_BT_PA
	      vreg = vreg_get(0, vreg_bt);
	      if ((rc = vreg_disable(vreg)))
		{printk(KERN_ERR "BLUETOOTH: %s: vreg(%s) disable failed (%d)\n", __func__,vreg_bt, rc);}
#ifndef CONFIG_MACH_MOT
}
#endif
	      
	      msleep_interruptible(100);
	    }
	  
	  power_is_on = 0;
	  printk(KERN_INFO "BLUETOOTH: bt_power: OFF\n");
	}
    }

#if defined(CONFIG_MACH_MOT)
	return 0;
#else
	return 1;
#endif
}

//////////////////////////////////////////////////////////////////////
static int __init bt_power_init(void)
{
  //  if (create_proc_read_entry("bt_power", 0, NULL, bt_power_read_proc,NULL) == 0) 
  proc_entry = create_proc_entry("bt_power", 0667, NULL);
  if(!proc_entry)
    {
      printk(KERN_ERR "BLUETOOTH: Registration of proc \"bt_power\" file failed\n");
      return(-ENOMEM);
    }

  proc_entry->read_proc  = bt_power_read_proc;
  proc_entry->write_proc = bt_power_write_proc;

  printk(KERN_INFO "BLUETOOTH: /proc/bt_power created\n");

  {
    proc_entry = create_proc_entry("bt_stat", 0666, NULL);
    if(!proc_entry)
      {
	printk(KERN_ERR "BLUETOOTH: Registration of proc \"bt_stat\" file failed\n");
	return(-ENOMEM);
      }
    
    proc_entry->read_proc  = bt_stat_read_proc;
    proc_entry->write_proc = 0;
    
    printk(KERN_INFO "BLUETOOTH: /proc/bt_stat created\n");
  }

  {
    proc_entry = create_proc_entry("bt_status", 0666, NULL);
    if(!proc_entry)
      {
	printk(KERN_ERR "BLUETOOTH: Registration of proc \"bt_status\" file failed\n");
	return(-ENOMEM);
      }
    
    proc_entry->read_proc  = bt_status_read_proc;
    proc_entry->write_proc = 0;
    
    printk(KERN_INFO "BLUETOOTH: /proc/bt_status created\n");
  }

  {
    proc_entry = create_proc_entry("bt_irq", 0666, NULL);
    if(!proc_entry)
      {
       printk(KERN_ERR "BLUETOOTH: Registration of proc \"bt_irq\" file failed\n");
       return(-ENOMEM);
      }
    
    proc_entry->read_proc  = bt_irq_read_proc;
    proc_entry->write_proc = 0;
    
    printk(KERN_INFO "BLUETOOTH: /proc/bt_irq created\n");
  }

  { // turn off power to BT and hold it in reset
    struct vreg *vreg;
    int rc;

    gpio_set_value (BT_RESET, 0);	// put BT into reset
    gpio_set_value (INTERNAL_BT_VREG, 0);	// turn off internal BT VREG

    if(!gpio_get_value(WLAN_REG_ON_SIGNAL))
      {
#ifndef CONFIG_MACH_MOT
if
(
	  !machine_is_calgary() &&
	  !machine_is_msm7627_pittsburgh()
)
{
#endif
	// Turn off VREG_BT_PA
	vreg = vreg_get(0, vreg_bt);
	if ((rc = vreg_disable(vreg)))
	  {printk(KERN_ERR "BLUETOOTH: %s: vreg(%s) disable failed (%d)\n", __func__,vreg_bt, rc);}
#ifndef CONFIG_MACH_MOT
}
#endif
	
	msleep_interruptible(100);
      }

    printk(KERN_INFO "BLUETOOTH: bt_power: Bluetooth power is off, and BT module is in reset.\n");
    
    power_is_on = 0;
  }
  
  return 0;
}

//////////////////////////////////////////////////////////////////////
module_init(bt_power_init);

static void __exit bt_power_exit(void)
{
  remove_proc_entry("bt_power", NULL);
}

//////////////////////////////////////////////////////////////////////
module_exit(bt_power_exit);

//////////////////////////////////////////////////////////////////////
MODULE_LICENSE("GPL");
MODULE_AUTHOR("John Zafiris <John.Zafiris@motorola.com>");
MODULE_DESCRIPTION("\"bt_power\" Bluetooth power control for the Broadcom BCM4325");
MODULE_VERSION("1.0");

//////////////////////////////////////////////////////////////////////
