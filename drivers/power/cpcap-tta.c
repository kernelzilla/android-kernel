/*
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */


#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <mach/gpio.h>

#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/uaccess.h>

#define SHOLEST_TTA_CHRG_DET_N_GPIO  34
#define TIME_FOR_GPIO_HIGH          100
#define TTA_IRQ_NAME "tta IRQ"

enum cpcap_tta_det_state {
   TTA_NONE,
   TTA_CONFIG,
   TTA_ATTACHED,
   TTA_DETACHED,
   FTM_CABLE
};

struct read_sense_data {
  unsigned short clear_int:1;
  unsigned short dplus:1;
  unsigned short dminus:1;
  unsigned short vbus_4v4:1;
};

struct cpcap_tta_det_data {
  struct cpcap_device *cpcap;
  struct read_sense_data sense;
  struct delayed_work work;
  enum cpcap_tta_det_state state;
  unsigned char is_xcvr_enabled;
  unsigned char gpio_val;
};

static struct cpcap_device *misc_cpcap;

static int cpcap_tta_ioctl(struct inode *inode,
		struct file *file,
		unsigned int cmd,
		unsigned long arg);
static int cpcap_tta_open(struct inode *inode, struct file *file);
static int cpcap_tta_chgr_probe(struct platform_device *pdev);
static int cpcap_tta_chgr_remove(struct platform_device *pdev);

static const struct file_operations tta_fops = {
  .owner = THIS_MODULE,
  .open = cpcap_tta_open,
  .ioctl = cpcap_tta_ioctl,
};

static struct miscdevice tta_dev = {
  .minor  = MISC_DYNAMIC_MINOR,
  .name = "cpcap_tta",
  .fops = &tta_fops,
};

static int cpcap_tta_open(struct inode *inode, struct file *file)
{
  return 0;
}

static int cpcap_tta_ioctl(struct inode *inode,
     struct file *file, unsigned int cmd, unsigned long arg)
{
  int retval = 0;

  switch (cmd) {
  case CPCAP_IOCTL_TTA_READ_STATUS:
    {
      unsigned char gpio_val;
      unsigned short vbus;
      enum cpcap_tta_state state;

      disable_tta();
      enable_tta();
      mdelay(10);
      gpio_val = gpio_get_value(SHOLEST_TTA_CHRG_DET_N_GPIO);
      retval = cpcap_regacc_read(misc_cpcap, CPCAP_REG_INTS2, &vbus);
      if (retval)
	return retval;

      vbus  = ((vbus & CPCAP_BIT_VBUSVLD_S) ? 1 : 0);

      if (!gpio_val) {
		if (!vbus)
			state = TTA_DETECTED;
		else
			state = TTA_NOT_DETECTED;
      } else {
      state = TTA_NOT_DETECTED;
      }

      if (copy_to_user((void *)arg,
		(void *)&state, sizeof(state)))
	retval = -EFAULT;

    }
    break;

  default:
    return -EINVAL;
    break;
  }
  return retval;
}

static void set_transceiver(struct cpcap_tta_det_data *data)
{
  cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2,
   (data->is_xcvr_enabled ?
		CPCAP_BIT_USBXCVREN : 0),
		CPCAP_BIT_USBXCVREN);
}

static int get_sense(struct cpcap_tta_det_data *data)
{
  int retval = -EFAULT;
  unsigned short value;
  struct cpcap_device *cpcap;

  if (!data)
		return -EFAULT;
  cpcap = data->cpcap;

  retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS2, &value);
  if (retval)
		return retval;

  data->sense.vbus_4v4  = ((value & CPCAP_BIT_VBUSVLD_S) ? 1 : 0);

  retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS4, &value);
  if (retval)
		return retval;

  /* Clear ASAP after read. */
  if (data->sense.clear_int) {
	retval = cpcap_regacc_write(cpcap, CPCAP_REG_INT1,
				     CPCAP_BIT_DP_I,
				     CPCAP_BIT_DP_I);

	data->sense.clear_int = 0;
  }
  if (retval)
		return retval;

  data->sense.dplus    = ((value & CPCAP_BIT_DP_S) ? 1 : 0);
  data->sense.dminus   = ((value & CPCAP_BIT_DM_S) ? 1 : 0);

  return 0;
}

static void tta_detection_work(struct work_struct *work)
{
  struct cpcap_tta_det_data *data =
		container_of(work, struct cpcap_tta_det_data, work.work);

  unsigned short reg_mask;

  switch (data->state) {
  case TTA_NONE:
      cpcap_irq_mask(data->cpcap, CPCAP_IRQ_DPI);
      data->gpio_val = gpio_get_value(SHOLEST_TTA_CHRG_DET_N_GPIO);
      if (!(data->gpio_val)) {
				data->is_xcvr_enabled = 1;
				set_transceiver(data);
      }
      data->state = TTA_CONFIG;
      schedule_delayed_work(&data->work, msecs_to_jiffies(100));
      break;

  case TTA_CONFIG:
      get_sense(data);

	if (!(data->gpio_val)) {
		if (!(data->sense.vbus_4v4))
			data->state = TTA_ATTACHED;
		else
			data->state = FTM_CABLE;

	schedule_delayed_work(&data->work, msecs_to_jiffies(0));
	} else {
	data->state = TTA_NONE;
	}

      break;

  case TTA_ATTACHED:
     cpcap_regacc_read(data->cpcap, CPCAP_REG_VUSBC, &reg_mask);
     if ((reg_mask & (CPCAP_BIT_VUSB_MODE0 | CPCAP_BIT_VUSB_MODE1 |
					CPCAP_BIT_VUSB_MODE2)) == 0) {
			cpcap_regacc_write(data->cpcap, CPCAP_REG_VUSBC,
				(CPCAP_BIT_VUSB_MODE0 | CPCAP_BIT_VUSB_MODE1) ,
				(CPCAP_BIT_VUSB_MODE0 | CPCAP_BIT_VUSB_MODE1 |
				CPCAP_BIT_VUSB_MODE2));
			data->state = TTA_ATTACHED;
     }

      cpcap_regacc_read(data->cpcap, CPCAP_REG_USBC2, &reg_mask);
      data->is_xcvr_enabled = ((reg_mask & CPCAP_BIT_USBXCVREN) ? 1 : 0);
      if (!(data->is_xcvr_enabled)) {
				data->is_xcvr_enabled = 1;
				set_transceiver(data);
      }

      cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
				     CPCAP_BIT_PU_SPI,
				     CPCAP_BIT_PU_SPI);
	    cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
				    CPCAP_BIT_DM1K5PU,
				    (CPCAP_BIT_DP150KPU | CPCAP_BIT_DP1K5PU |
				     CPCAP_BIT_DM1K5PU | CPCAP_BIT_DPPD |
				     CPCAP_BIT_DMPD));

      get_sense(data);

      if ((data->sense.dplus) == (data->sense.dminus)) {
				data->state = TTA_ATTACHED;
				cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_DPI);
      } else {
      data->state = TTA_DETACHED;
      schedule_delayed_work(&data->work, msecs_to_jiffies(0));
      }
      break;

  case FTM_CABLE:
      data->state = TTA_NONE;
      disable_tta();
      break;

  case TTA_DETACHED:
      cpcap_irq_mask(data->cpcap, CPCAP_IRQ_DPI);
      data->state = TTA_NONE;
      disable_tta();
      enable_tta();

      cpcap_regacc_write(data->cpcap, CPCAP_REG_VUSBC,
					0 ,
				(CPCAP_BIT_VUSB_MODE0 | CPCAP_BIT_VUSB_MODE1 |
				CPCAP_BIT_VUSB_MODE2));

      data->is_xcvr_enabled = 0;
      set_transceiver(data);

      cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
				    CPCAP_BIT_DP150KPU,
				    (CPCAP_BIT_DP150KPU | CPCAP_BIT_DP1K5PU |
				     CPCAP_BIT_DM1K5PU | CPCAP_BIT_DPPD |
				     CPCAP_BIT_DMPD));
      break;

  default:
      data->state = TTA_NONE;
      schedule_delayed_work(&data->work, msecs_to_jiffies(0));
      break;
   }
}

irqreturn_t isr_handler(int irq, void *dev_id, struct pt_regs *regs)
{
  struct cpcap_tta_det_data *data;
  data = (struct cpcap_tta_det_data *) dev_id;
  schedule_delayed_work(&data->work, msecs_to_jiffies(0));
  return IRQ_HANDLED;
}

static void int_handler(enum cpcap_irqs int_event, void *data)
{
  struct cpcap_tta_det_data *tta_det_data = data;
  tta_det_data->sense.clear_int = 1;
  schedule_delayed_work(&(tta_det_data->work), 0);
}

void enable_tta(void)
{
  mdelay(TIME_FOR_GPIO_HIGH);
  gpio_direction_input(SHOLEST_TTA_CHRG_DET_N_GPIO);
}

void disable_tta(void)
{
  gpio_direction_output(SHOLEST_TTA_CHRG_DET_N_GPIO, 1);
}

static int cpcap_tta_chgr_probe(struct platform_device *pdev)
{
  int retval = 0;
  struct cpcap_tta_det_data *data;

  if (pdev->dev.platform_data == NULL) {
	dev_err(&pdev->dev, "no platform_data\n");
	return -EINVAL;
  }

  data = kzalloc(sizeof(*data), GFP_KERNEL);
  if (!data)
	return -ENOMEM;

  data->cpcap = pdev->dev.platform_data;
  data->state = TTA_NONE;
  platform_set_drvdata(pdev, data);
  INIT_DELAYED_WORK(&data->work, tta_detection_work);

  misc_cpcap = data->cpcap;  /* kept for misc device */
  retval = misc_register(&tta_dev);
  if (retval)
	return -EFAULT;

  if (gpio_request(SHOLEST_TTA_CHRG_DET_N_GPIO, "tta_chrg_cntr") < 0)
	return -EBUSY;

  set_irq_type(OMAP_GPIO_IRQ(SHOLEST_TTA_CHRG_DET_N_GPIO),
			IRQ_TYPE_EDGE_FALLING);
  retval |= request_irq(
			OMAP_GPIO_IRQ(SHOLEST_TTA_CHRG_DET_N_GPIO),
			(void *)isr_handler,
			IRQF_DISABLED, TTA_IRQ_NAME,
			data);

  set_irq_type(CPCAP_IRQ_DPI, IRQ_TYPE_EDGE_FALLING);
  retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_DPI,
				     int_handler, data);

  if (retval != 0) {
		dev_err(&pdev->dev, "Initialization Error\n");
		retval = -ENODEV;
		goto free_mem;
	}

  dev_info(&pdev->dev, "TTA charger detection device probed\n");

  tta_detection_work(&(data->work.work));
  return 0;
free_mem:
  kfree(data);
  return retval;
}

static int cpcap_tta_chgr_remove(struct platform_device *pdev)
{
  misc_deregister(&tta_dev);
  free_irq(OMAP_GPIO_IRQ(SHOLEST_TTA_CHRG_DET_N_GPIO), 0);
  gpio_free(SHOLEST_TTA_CHRG_DET_N_GPIO);

  return 0;
}

static struct platform_driver cpcap_tta_chgr_driver = {
  .probe    = cpcap_tta_chgr_probe,
  .remove   = cpcap_tta_chgr_remove,
  .driver   = {
    .name = "cpcap_tta_charger",
    .owner  = THIS_MODULE,
  },
};

static int __init cpcap_tta_chgr_init(void)
{
  return platform_driver_register(&cpcap_tta_chgr_driver);
}
module_init(cpcap_tta_chgr_init);

static void __exit cpcap_tta_chgr_exit(void)
{
  platform_driver_unregister(&cpcap_tta_chgr_driver);
}
module_exit(cpcap_tta_chgr_exit);

MODULE_ALIAS("platform:cpcap_tta_chgr");
MODULE_DESCRIPTION("CPCAP TTA Charger Device driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");

