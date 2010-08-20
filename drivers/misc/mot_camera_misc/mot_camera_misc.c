/*

 JK: a more appropriate header will be added later

 Revision History:

 Modification                         Tracking
 Author                 Date          Number     Description of Changes
 Joe Kreidler (qa1818)  23Mar2009                Initial Version
*/

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>

#include <../arch/arm/plat-omap/include/mach/gpio.h>
#include <../arch/arm/plat-omap/include/mach/resource.h>
#include <../arch/arm/plat-omap/include/mach/dt_path.h>
#include <../arch/arm/include/asm/prom.h>


#include "mot_camera_misc.h"
#include <../drivers/media/video/oldisp/isp.h>
#include <../drivers/media/video/oldisp/ispreg.h>

#include <linux/gpio_mapping.h>
#include <linux/of.h>


#define CONTROL_PADCONF_CAM_FLD   0x48002114
#define CONTROL_PADCONF_CAM_XCLKA 0x48002110

static void _camera_lines_lowpower_mode(void);
static void _camera_lines_func_mode(void);

static int camera_dev_open (struct inode *inode, struct file *file);
static int camera_dev_release (struct inode *inode, struct file *file);
static int camera_dev_ioctl (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);


static struct file_operations camera_dev_fops = {
    .open           = camera_dev_open,
    .release        = camera_dev_release,
    .ioctl          = camera_dev_ioctl
};

static struct miscdevice cam_misc_device0 = {
    .minor          = MISC_DYNAMIC_MINOR,
    .name           = "camera0",
    .fops           = &camera_dev_fops
};

static int gpio_reset ;
static int gpio_powerdown ;
int isp_count_local = 0;
unsigned int is_smart_cam = 0;

#define GPIO_SENSOR_NAME	"camera_misc"

#define DEBUG 0

#define err_print(fmt, args...) printk(KERN_ERR "fun %s "fmt"\n", __FUNCTION__, ##args)

#if DEBUG > 0

#define dbg_print(fmt, args...) printk(KERN_INFO "fun %s "fmt"\n", __FUNCTION__, ##args)

#if DEBUG > 1
#define ddbg_print(fmt, args...) printk(KERN_INFO "fun %s "fmt"\n", __FUNCTION__, ##args)
#else
#define ddbg_print(fmt, args...) ;
#endif

#else

#define dbg_print(fmt, args...)  ;
#define ddbg_print(fmt, args...) ;

#endif

static struct regulator *regulator;

static int bHaveResetGpio = 0;
static int bHavePowerDownGpio = 0;
static int camera_dev_open(struct inode *inode, struct file *file)
{
    return 0;
}


static int camera_dev_release(struct inode *inode, struct file *file)
{
    return 0;
}


static int camera_dev_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)

{

    int rc=0;
    struct isp_sysc isp_sysconfig;

    if (!bHaveResetGpio) {
      ddbg_print("Requesting reset gpio\n");
      rc = gpio_request(gpio_reset, "camera reset");
      if (!rc) {
	bHaveResetGpio = 1;
      }
    }
    if (!bHavePowerDownGpio) {
      ddbg_print("Requesting powerdown gpio\n");
      rc = gpio_request(gpio_powerdown, "camera powerdown");
      if (!rc) {
	bHavePowerDownGpio = 1 ;
      }
    }
    ddbg_print( "camera ioctl cmd = %u, arg = %lu\n", cmd, arg);

    switch(cmd)
    {
        case CAMERA_RESET_WRITE:  
	  if (bHaveResetGpio)
            {
	      gpio_direction_output(gpio_reset, 0);
	      gpio_set_value(gpio_reset, (arg ? 1 : 0));
	      dbg_print("CAMERA_MISC set RESET line to %u\n", (arg ? 1 : 0));
            }
	  
	  if (!bHaveResetGpio)
            {
              err_print ("CAMERA_MISC: gpio_request () failed. rc = %d; cmd = %d; arg = %lu", rc, cmd, arg);
              return -EIO;
            }
            break;

        case CAMERA_POWERDOWN_WRITE:
            if (bHavePowerDownGpio)
            {
	      gpio_direction_output(gpio_powerdown, 0);
                if (0 == arg)
                {
		  gpio_set_value(gpio_powerdown, 0);
                }
                else
                {
		  gpio_set_value(gpio_powerdown, 1);
                }
                dbg_print( "CAMERA_MISC set POWERDOWN line to %u\n", (arg ? 1 : 0));
            }
            if (!bHavePowerDownGpio)
            {
              err_print ("CAMERA_MISC: gpio_request () failed. rc = %d; cmd = %u; arg = %lu", rc, cmd, arg);
              return -EIO;
            }
            break;

        case CAMERA_CLOCK_DISABLE:
	  //powerup the isp/cam domain on the 3410
	  if (isp_count_local == 0) {
	    isp_get();
	    isp_count_local++;
	  }
	  isp_sysconfig.reset = 0;
	  isp_sysconfig.idle_mode = 1;
	  isp_power_settings(isp_sysconfig);
	  omap_writel((omap_readl(ISP_SYSCONFIG)&0xfffffffe)|1,ISP_SYSCONFIG);
	  while (omap_readl(ISP_SYSSTATUS)!=1) {
	    dbg_print("CAMERA_MISC Waiting for reset done ISP_SYSSTATUS=%d",omap_readl(ISP_SYSSTATUS));
	    msleep(10);
	  }
	  isp_set_xclk(0, 0);

	  /* Need to make sure that all encounters of the
	     isp clocks are disabled*/

	  while (isp_count_local > 0) {
	    isp_put();
	    isp_count_local--;
	  }
	  dbg_print( "CAMERA_MISC turned off MCLK done\n");
	  break;

        case CAMERA_CLOCK_ENABLE:
	  //powerup the isp/cam domain on the 3410
	  if (isp_count_local == 0) {
	    isp_get();
	    isp_count_local++;
	  }

	  isp_sysconfig.reset = 0;
	  isp_sysconfig.idle_mode = 1;
	  isp_power_settings(isp_sysconfig);

	  omap_writel((omap_readl(ISP_SYSCONFIG)&0xfffffffe)|1,ISP_SYSCONFIG);
	  while (omap_readl(ISP_SYSSTATUS)!=1) {
	    dbg_print("CAMERA_MISC Waiting for reset done ISP_SYSSTATUS=%d",omap_readl(ISP_SYSSTATUS));
	    msleep(10);
	  }
	  isp_set_xclk(arg, 0);
	  dbg_print( "CAMERA_MISC set MCLK to %d\n", (int) arg);
	  break;
        case CAMERA_AVDD_POWER_ENABLE:
	case CAMERA_AVDD_POWER_DISABLE:


	   if (arg == 1)
	     {
	       /* turn on digital power */
	       if (regulator != NULL) {
		 err_print("Already have regulator\n");
	       } else {
		 regulator = regulator_get(NULL, "vcam");
		 if (IS_ERR(regulator)) {
		   err_print("Cannot get vcam regulator, err=%ld\n",PTR_ERR(regulator));
		   return PTR_ERR(regulator);
		 }
	       }
		   /* No longer enabling regulator since VCAM is left on all the time for Ruth */
		   /*
	       if (regulator_enable(regulator) != 0) {
		 err_print("Cannot enable vcam regulator\n");
		 return -EIO;
	       }
		   */
		   
		   regulator_set_mode(regulator, REGULATOR_MODE_NORMAL);
	       _camera_lines_func_mode();
	     }
	   else
	     {
	       /* Turn off power */
	       if (regulator != NULL) {
		   
		   regulator_set_mode(regulator, REGULATOR_MODE_STANDBY);
		 regulator_disable(regulator);
		 regulator = NULL;
	       } else {
		 err_print("Regulator for vcam is notinitialized\n");
		 return -EIO;
	       }
	       _camera_lines_lowpower_mode();
	     }
	   break;
    default:
            err_print( "CAMERA_MISC received unsupported cmd; cmd = %u\n", cmd);
            return -EIO;
            break;
    }

    return 0;
}

static int __init camera_misc_init(void)
{
	struct device_node *feat_node;
	const void *feat_prop;
	
	ddbg_print("camera misc init\n");
	
	/* Check sensor Type */
	feat_node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);
	if (NULL != feat_node) {
		feat_prop = of_get_property(feat_node,
					"feature_smart_cam", NULL);
		if (NULL != feat_prop) {
			is_smart_cam = *(u8 *)feat_prop;
			printk(KERN_INFO "feature_smart_cam %d\n", is_smart_cam) ;
		}
		
	}
	
	if ( is_smart_cam ) {
    	gpio_reset = get_gpio_by_name("gpio_cam_reset");
    	gpio_powerdown = get_gpio_by_name("gpio_cam_pwdn");	
    	ddbg_print("gpio_cam_reset=%d gpio_cam_pwdn=%d", gpio_reset, gpio_powerdown);

		_camera_lines_lowpower_mode();

		// Using gpio 64 for CAM_PWRDOWN PIN
		/* Standby needs to be high */
		if ( !gpio_request(gpio_powerdown, "camera powerdown") )
		{
			gpio_direction_output(gpio_powerdown, 0);
			gpio_set_value(gpio_powerdown, 1);

			// Do not free gpio so that it cannot be overwritten
			bHavePowerDownGpio = 1 ;
		} 		
    }

    if(misc_register( &cam_misc_device0 ))
    {
        err_print( "error in register camera misc device!\n" );
        return -EIO;
    }
    return 0;
}

static void __exit camera_misc_exit(void)
{
  ddbg_print("camera misc exit\n");
  misc_deregister(&cam_misc_device0);
  if (!bHaveResetGpio) {
    gpio_free(gpio_reset);
    bHaveResetGpio = 0;
  }
  if (!bHavePowerDownGpio) {
    gpio_free(gpio_powerdown);
    bHavePowerDownGpio = 0;
  }
}


void _camera_lines_lowpower_mode(void)
{
	omap_writew(0x0007, 0x4800210c);   /* HSYNC */
	omap_writew(0x0007, 0x4800210e);   /* VSYNC */
	omap_writew(0x0007, 0x48002110);   /* MCLK */
	omap_writew(0x0007, 0x48002112);   /*PCLK */
	omap_writew(0x3A04, 0x48002114 );  /* GPIO 98 CAMERA_RESET */
	omap_writew(0x0007, 0x4800211a);   /* CAM_D2 */
	omap_writew(0x0007, 0x4800211c);   /* CAM_D3 */
	omap_writew(0x0007, 0x4800211e);   /* CAM_D4 */ 
	omap_writew(0x0007, 0x48002120);   /* CAM_D5 */
	omap_writew(0x001F, 0x48002122);   /* CAM_D6 */
	omap_writew(0x001F, 0x48002124);   /* CAM_D7 */
	omap_writew(0x001F, 0x48002126);   /* CAM_D8 */
	omap_writew(0x001F, 0x48002128);   /* CAM_D9 */
}

void _camera_lines_func_mode(void)

{
	omap_writew(0x0118, 0x4800210c);  /* HSYNC */ 
	omap_writew(0x0118, 0x4800210e);  /* VSYNC */
	omap_writew(0x0000, 0x48002110);  /* MCLK */
	omap_writew(0x0118, 0x48002112);  /*PCLK */
	omap_writew(0x0004, 0x48002114);  /* GPIO 98 CAMERA_RESET */
	omap_writew(0x0100, 0x4800211a);  /* CAM_D2 */
	omap_writew(0x0100, 0x4800211c);  /* CAM_D3 */
	omap_writew(0x0100, 0x4800211e);  /* CAM_D4 */
	omap_writew(0x0100, 0x48002120);  /* CAM_D5 */
	omap_writew(0x0100, 0x48002122);  /* CAM_D6 */
	omap_writew(0x0100, 0x48002124);  /* CAM_D7 */
	omap_writew(0x0100, 0x48002126);  /* CAM_D8 */
	omap_writew(0x0100, 0x48002128);  /* CAM_D9 */
}

module_init(camera_misc_init);
module_exit(camera_misc_exit);
