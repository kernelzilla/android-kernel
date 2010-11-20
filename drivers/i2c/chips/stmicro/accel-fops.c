/**
 *	Morrison accelerometer I2C protocol driver
 *
 *	Copyright (C) 2008  Motorola, Inc.
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation version 2 of the License.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	GNU General Public License <http://www.gnu.org/licenses/gpl-2.0.html>
 */

#include <linux/lis331dlh.h>
#include "accel-main.h"

static accel_client_t *accel_find_client_by_pid (pid_t pid)
{
	accel_client_t *client = accel_info.clients;

	while (client != NULL) {
		if (client->pid == pid) return client;
		client = client->next;
	}
	return NULL;
}

static accel_client_t *accel_add_client (pid_t pid)
{
	accel_client_t *prev, *client;

	prev = accel_info.clients;
	client = kzalloc (sizeof(accel_client_t), GFP_KERNEL);
	if (client == NULL) 
		return NULL;
	client->pid = pid;
	client->next = prev;
	if (prev != NULL) 
		prev->prev = client;
	accel_info.clients = client;
	accel_info.num_clients++;

	vprintk ("added client pid %d", pid);

	return client;
}

static void accel_remove_client (pid_t pid)
{
	accel_client_t *prev, *next, *client;
	
	client = accel_find_client_by_pid (pid);
	if (client == NULL)
		return;

	prev = client->prev;
	next = client->next;
	if (prev != NULL) {
		prev->next = next;
		if (next != NULL)
			next->prev = prev;
	}else {
		if (next != NULL)
			next->prev = NULL;
		accel_info.clients = next;
	}
	accel_info.num_clients--;
	kfree (client);

	vprintk ("removed client pid %d", pid);
}

/*
 * Following function requires caller to hold accelerometer's mutex !!!!!
 */
static void accel_insert_file_private (accel_file_private_t *priv)
{
	accel_file_private_t *prev;

	prev = accel_info.privates;
	if (prev != NULL) 
		prev->prev = priv;
	priv->next = prev;
	accel_info.privates = priv;
}

/*
 * Following function requires caller to hold accelerometer's mutex !!!!!
 */
static void accel_remove_file_private (accel_file_private_t *priv)
{
	accel_file_private_t *prev, *next;
	
	next = priv->next;
	prev = priv->prev;

	if (prev != NULL) {
		prev->next = next;
		if (next != NULL)
			next->prev = prev;
	}else {
		if (next != NULL)
			next->prev = NULL;
		accel_info.privates = next;
	}
}

accel_file_private_t *accel_find_private_by_pid (pid_t pid)
{
	accel_file_private_t *priv = accel_info.privates;

	while (priv != NULL) {
		if (priv->mypid == pid) return priv;
		priv = priv->next;
	}
	return NULL;
}


void accel_clients_notify (unsigned long event, struct input_event *data)
{
	accel_file_private_t *priv;
	int done=0;

	//mutex_lock (&accel_info.mlock); 
	priv = accel_info.privates;
	while (priv) {
		if (priv->events_mask & event) {

			done++;	/* count # of clients notified */

			down (&priv->sem);
			priv->iev = *data;
			priv->status |= ACCEL_ST_DATA_READY;
			up (&priv->sem);

			/* finally, awake any readers */
			wake_up_interruptible (&priv->readq);
		}
		priv = priv->next;
	}
	//mutex_unlock (&accel_info.mlock);

	vprintk ("%d clients notified about event 0x%08lx", done, event);
}

static int accel_lookup_private_mask (accel_file_private_t *skip, unsigned long bit)
{
	accel_file_private_t *priv;
	int found=0;

	priv = accel_info.privates;
	while (priv) {
		if (priv != skip && 
		   (priv->events_mask & bit)) {
			++found;
			break;
		}
		priv = priv->next;
	}
	return found;
}

static void accel_update_global_mask (unsigned long bit, int toggle, int lookup, accel_file_private_t *skip)
{	
	mutex_lock (&accel_info.mlock);
	if (lookup < 0)
		lookup = accel_lookup_private_mask (skip, bit);
	if (lookup == 0) {

		if (toggle == TOGGLE_ON)
			accel_events_mask |= bit;
		else	accel_events_mask &= ~bit;

		vprintk ("global events mask 0x%08lx", accel_events_mask);
	}
	mutex_unlock (&accel_info.mlock);
}

#define KEYWORD(a,r1,r1v,r2,r2v,r3,r3v,r4,r4v,r5,r5v)  {r1v,r2v,r3v,r4v,r5v},
static unsigned char accel_ctrl_regs_configs[ACCEL_MODE_MAX][5]={ACCEL_CTRL_REGS_CONFIGS};
#undef KEYWORD

#define KEYWORD(a,p1,irq,p2,p2v,p3,p3v,p4,p4v)		irq,
static int accel_irq_assignment[ACCEL_MODE_MAX]={ACCEL_IRQ_CONFIGS};
#undef KEYWORD

#define KEYWORD(a,p1,p1v,p2,p2v,p3,p3v,p4,p4v)		{p2v,p3v,p4v},
static unsigned char accel_irq_configs[ACCEL_MODE_MAX][3]={ACCEL_IRQ_CONFIGS};
#undef KEYWORD

static int accel_mask2mode (unsigned long bit)
{
	int mode = -1;
	switch (bit) {
	 case ACCEL_EV_RAW_DATA : 
		mode = ACCEL_MODE_RAW_DATA; break;
	 case ACCEL_EV_SCREEN_ORIENT : 
		mode = ACCEL_MODE_SCREEN_ORIENT; break;
         case ACCEL_EV_TAP | ACCEL_EV_DOUBLE_TAP : 
		mode = ACCEL_MODE_TAPPING; break;
	 case ACCEL_EV_THROW : 
		mode = ACCEL_MODE_THROW; break;
	 case ACCEL_EV_SWING : 
		mode = ACCEL_MODE_SWING; break;
	}
	return mode;
}

static void accel_apply_config (int mode)
{
	unsigned char *config = accel_ctrl_regs_configs [mode];
	/* in order to disable configuration, it's enough to setup reg1 to 0 */
	if (*config) {
		unsigned char *config_irq = accel_irq_configs [mode];
		/* TODO: is it neccessary to turn power off */
		//accel_i2c_set_power_mode (PWR_OFF);

		dprintk ("mode: %d r1: 0x%02x r2: 0x%02x r3: 0x%02x r4: 0x%02x r5: 0x%02x",
				mode, *config, *(config+1), *(config+2), *(config+3), *(config+4));

		accel_i2c_write_register (ACCEL_I2C_PORT_REG2, *(config+1));
		accel_i2c_write_register (ACCEL_I2C_PORT_REG3, *(config+2));
		accel_i2c_write_register (ACCEL_I2C_PORT_REG4, *(config+3));
		accel_i2c_write_register (ACCEL_I2C_PORT_REG5, *(config+4));

		dprintk ("irq: %d cfg: 0x%02x ths: 0x%02x dur: 0x%02x",
				accel_irq_assignment [mode], *config_irq, *(config_irq+1), *(config_irq+2));

		if (accel_irq_assignment [mode] == 1) {
			accel_i2c_write_register (ACCEL_I2C_PORT_INT1_CFG, *config_irq);
			accel_i2c_write_register (ACCEL_I2C_PORT_INT1_THS, *(config_irq+1));
			accel_i2c_write_register (ACCEL_I2C_PORT_INT1_DUR, *(config_irq+2));
		}else {
			accel_i2c_write_register (ACCEL_I2C_PORT_INT2_CFG, *config_irq);
			accel_i2c_write_register (ACCEL_I2C_PORT_INT2_THS, *(config_irq+1));
			accel_i2c_write_register (ACCEL_I2C_PORT_INT2_DUR, *(config_irq+2));
		}

		accel_i2c_write_register (ACCEL_I2C_PORT_REG1, *config);
	}
}

static void accel_cleanup_config (void)
{
	accel_i2c_set_power_mode (PWR_OFF);
}


static void accel_change_config (unsigned long bit, int toggle, accel_file_private_t *skip)
{
	int found, mode = accel_mask2mode (bit);

	if (mode == -1) {
		dprintk ("unknown event 0x%08lx", bit);
		return;
	}

	/* we need to know if there are others clients waiting for the same event */
	found = accel_lookup_private_mask (skip, bit);

	if (found == 0 && toggle == TOGGLE_ON) {
		/* turn on power */
		accel_apply_config (mode);
	}

	if (found == 0 && toggle == TOGGLE_OFF) {
		/* turn off power */
		accel_cleanup_config ();
	}

	/* finally update global events mask */
	accel_update_global_mask (bit, toggle, found, skip);
}

static void accel_update_private_mask (accel_file_private_t *priv, unsigned long new)
{
	int i, turned_on, turn_on;
	unsigned long bit_mask, old=priv->events_mask;

	for (i=ACCEL_BITS_IN_ULONG; i >= 0; i--) {

		bit_mask = 1 << i;

		if (old & bit_mask)	turned_on = 1;
		else			turned_on = 0;

		if (new & bit_mask)	turn_on = 1;
		else			turn_on = 0;

		if (!(turned_on | turn_on) ||
		     (turned_on && turn_on))	continue;

		dprintk ("bit: 0x%08lx turned_on: %d turn_on: %d", bit_mask, turned_on, turn_on);

		if (turned_on && ! turn_on) { 
			/* Turn Off */
			dprintk ("turn off bit 0x%08lx", bit_mask);
			accel_change_config (bit_mask, TOGGLE_OFF, priv);
		}

		if (! turned_on && turn_on) { 
			/* Turn On */
			dprintk ("turn on bit 0x%08lx", bit_mask);
			accel_change_config (bit_mask, TOGGLE_ON, priv);
		}
	}
}

/**
 * Open device
 *
 * @param inode
 * @param filp
 * @return 0
 */
static int accel_open(struct inode *inode, struct file *filp)
{
	accel_file_private_t *priv;
	accel_client_t *client;
	int rc;

	vprintk ("open request from pid %d", current->pid);

	priv = kzalloc (sizeof(accel_file_private_t), GFP_KERNEL);
	if (priv == NULL) 
		goto err_nomem;

	client = accel_find_client_by_pid (current->pid);
	if (client == NULL) {
		mutex_lock (&accel_info.mlock);
		client = accel_add_client (current->pid);
		mutex_unlock (&accel_info.mlock);
		if (client == NULL)
			goto err_nomem;
	}

	/* initialize read wait queue */
	init_waitqueue_head (&priv->readq);
	init_MUTEX (&priv->sem);

	down (&priv->sem);
	priv->access_flags |= ACCEL_FF_IS_CLIENT;
	priv->access_flags |= ACCEL_FF_IS_VALID;	
	priv->mypid = current->pid;	
	up (&priv->sem);

	filp->private_data = priv;

	mutex_lock (&accel_info.mlock);
	accel_insert_file_private (priv);
	client->file_private = (accel_file_private_t **)&filp->private_data;
	mutex_unlock (&accel_info.mlock);

	return nonseekable_open(inode, filp);

err_nomem:
	rc = -ENOMEM;
	return rc;
}


/**
 * Read implementation
 *
 * Reads the acceleration data (X,Y,Z) and copies them to user.
 *
 * @param f Kernel level file structure
 * @param buffer Userspace buffer
 * @param bytes Bytes requested
 * @param offset Offset requested
 */
static int accel_read(struct file *f, char __user *buffer, size_t bytes, long long *offset)
{
	accel_event_t iev;
	int nread = sizeof (accel_event_t);

	if (bytes < nread)
		return -ENOMEM;
#ifdef CONFIG_MACH_MOT
	accel_i2c_read_xyz(iev.ev_values);
#else
	if (accel_i2c_read_axis_data (iev.ev_values))
		return -EFAULT;
#endif

	iev.ev_code = ACCEL_EV_XYZ | EV_ABS;

	if (copy_to_user(buffer, &iev, nread))
		return -EFAULT;

	return nread;
}


/**
 * Write implementation
 *
 * Writes the acceleration data (X,Y,Z) provided to accelerometer input [data injection].
 * NOTE: No validity checks done!!!
 *
 * @param f Kernel level file structure
 * @param buffer Userspace buffer
 * @param bytes Bytes requested
 * @param offset Offset requested
 */
static int accel_write(struct file *f, const char __user *buffer, size_t bytes, long long *offset)
{
	accel_event_t aev;
	int screen=0, nread = sizeof (accel_event_t);
	struct input_dev *idev = accel_info.idev;

	if (bytes < nread)
		return -ENOMEM;

	if (copy_from_user(&aev, (char __user *)buffer, nread))
		return -EFAULT;

	dprintk ("ev_code 0x%04x", aev.ev_code);

	if ((aev.ev_code & 0xff00) == ACCEL_EV_ROTATION_ENABLE) {
	   // This is needed to deliver P/L chnage events directly to PhoneWindowManager
	   // NOTE: It matters only for events injection
	   dprintk ("rotation enable request");
	   if (idev) {
		input_report_switch (idev, 6, TOGGLE_ON);
		input_sync (idev);
	   }
	}else

	if ((aev.ev_code & 0xff00) == ACCEL_EV_ROTATION_DISABLE) {
	   // This is needed to disable delivery of P/L chnage events to PhoneWindowManager
	   // NOTE: It matters only for events injection
	   dprintk ("rotation disable request");
	   if (idev) {
		input_report_switch (idev, 6, TOGGLE_OFF);		
		input_sync (idev);
	   }
	} else {
	  // otherwise look what data came in
	  switch (aev.ev_code & 0xff) {
	   case EV_ABS : // inject x,y,x triplet
		input_report_abs (idev, ABS_X, aev.ev_values[0]);
		input_report_abs (idev, ABS_Y, aev.ev_values[1]);
		input_report_abs (idev, ABS_Z, aev.ev_values[2]);
		input_sync (idev);

		dprintk ("ev_values: %d:%d:%d ", aev.ev_values[0], aev.ev_values[1], aev.ev_values[2]);
			break;
	   case EV_SW : // inject 
		if (aev.ev_values[0] == ACCEL_ORIENT_NORMAL ||
		    aev.ev_values[0] == ACCEL_ORIENT_UPSIDE)
		    screen = ACCEL_SCREEN_PORTRAIT;
		else 
		    if (aev.ev_values[0] == ACCEL_ORIENT_TILTED_LEFT ||
			aev.ev_values[0] == ACCEL_ORIENT_TILTED_RIGHT)
			screen = ACCEL_SCREEN_LANDSCAPE;

		if (screen) {
			input_report_abs (idev, ABS_TILT_X, screen);
			input_report_abs (idev, ABS_TILT_Y, aev.ev_values[0]);
			input_sync (idev);

			dprintk ("screen orientation: %d 0x%08x", screen, aev.ev_values[0]);
			break;
		}
		dprintk ("invalid screen orientation");
	   default :
		return -EINVAL;
	  }
	}
	return nread;
}


/**
 * Ioctl implementation
 *
 * Device can be used only by one user process at the time
 * ioctl returns -EBUSY if device is already in use
 *
 * @param node File in /proc
 * @param f Kernel level file structure
 * @param cmd Ioctl command
 * @param arg Ioctl argument
 * @return 0 in success, or negative error code
 */
static int accel_ioctl(struct inode *node, struct file *filp, unsigned int cmd, unsigned long arg)
{
	accel_file_private_t *priv = filp->private_data;
	int retval = 0, err = 0;
	unsigned int ui_data;
	unsigned long ul_data;
    void __user *argp = (void __user *)arg;
    int mode;
    unsigned enable;
    int current_mode;
    int interval, odr;

	//return -ENOSYS;

	//if (_IOC_TYPE(cmd) != ACCEL_IOC_MAGIC) return -ENOTTY;
	//if (_IOC_NR(cmd) > ACCEL_IOC_MAXNR) return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;

	switch (cmd) {
        case LIS331DLH_IOCTL_SET_ENABLE:
            if (copy_from_user(&mode, argp, sizeof(mode)))
                return -EFAULT;

            printk (KERN_INFO "%s: SET_ENABLE: flags = 0x%x\n", __func__, mode);
            if (mode != 0) {
                if (!((mode & LIS331DLH_MODE_ACCEL) || 
                      (mode & LIS331DLH_MODE_ROTATE))) {
                    printk (KERN_ERR "%s: invalid mode %d\n",
                        __FUNCTION__, mode);
                    return -EINVAL;
                }
            }

            enable = mode >> 8;
            mode &= 0xFF;
            current_mode = atomic_read(&accel_info.mode);
            if (enable) {
                mode = current_mode | mode;
            } else {
                mode = current_mode & (~mode);
            }

            printk (KERN_INFO "%s: SET_ENABLE %d => %d\n", 
                __func__, current_mode, mode);
            accel_i2c_set_config (mode, -1, -1, -1);
            break;

        case LIS331DLH_IOCTL_SET_DELAY:
            if (copy_from_user(&interval, argp, sizeof(interval)))
                return -EFAULT;
            if (interval < 3)
                odr = 3;
            else if (interval <= 10)
                odr = 2;
            else if (interval <= 20)
                odr = 1;
            else 
                odr = 0;
            printk (KERN_INFO "%s: SET_DELAY %dms, odr=%d\n", 
                __FUNCTION__, interval, odr);
            if (interval < 3)
                interval = 3;
            accel_i2c_set_config (-1, odr, -1, interval);
            break;

        case LIS331DLH_IOCTL_TRIG_ROTATION:
             retval = copy_from_user (&ui_data, (int __user *)arg, sizeof(ui_data));
             if (retval != 0)
                 return -EFAULT;
             dprintk ("ioctl: trigger rotation event ");
             if( accel_events_mask & ACCEL_EV_SCREEN_ORIENT )       
                 accel_irq_bottom_half_job(1);
             retval = SUCCESS;
             break;


		case IOCTL_ACCEL_SET_EVENTS :

			retval = copy_from_user (&ul_data, (int __user *)arg, sizeof(ul_data));
			if (retval != 0)
				return -EFAULT;

			accel_update_private_mask (priv, ul_data);

			down (&priv->sem);
			priv->events_mask |= ul_data;
			up (&priv->sem);

			dprintk ("events mask set 0x%08lx", priv->events_mask);

			retval = SUCCESS;
			break;

		case IOCTL_ACCEL_GET_EVENTS :

			down (&priv->sem);
			ul_data = priv->events_mask;
			up (&priv->sem);

			dprintk ("events mask get 0x%08lx", ul_data);

			retval = copy_to_user ((int __user *)arg, &ul_data, sizeof(ul_data));
			break;

		case IOCTL_ACCEL_CLR_EVENTS :

			retval = copy_from_user (&ul_data, (int __user *)arg, sizeof(ul_data));
			if (retval != 0)
				return -EFAULT;

			dprintk ("events mask clear 0x%08lx", ul_data);

			accel_update_private_mask (priv, ul_data);

			down (&priv->sem);
			priv->events_mask &= ~ul_data;
			up (&priv->sem);

			retval = SUCCESS;
			break;

		case IOCTL_ACCEL_GET_EVENT_DATA :
			/*
			dprintk ("ioctl: fetch event data");

			retval = -EFAULT;
			down (&priv->sem);
			if (priv->status & ACCEL_ST_DATA_READY)
				retval = copy_to_user ((int __user *)arg, &priv->iev, sizeof(priv->iev));
			priv->status &= ~ACCEL_ST_DATA_READY;
			up (&priv->sem);*/
			retval = -EFAULT;
			break;

#if 0
		case IOCTL_ACCEL_SET_PWR_MODE:
			retval = -EFAULT;
			break;
#endif

		case IOCTL_ACCEL_GET_PWR_MODE:
			retval = -EFAULT;
			break;

		case IOCTL_ACCEL_GET_REVID:
			retval = -EFAULT;
			break;

		case IOCTL_ACCEL_IRQ_CONFIG:
			retval = copy_from_user (&ui_data, (int __user *)arg, sizeof(ui_data));
			if (retval != 0)
				return -EFAULT;
			dprintk ("ioctl: irq config %s", ! ui_data ? "DISABLE" : "ENABLE");
			accel_configure_irqs (ui_data);
			retval = SUCCESS;
			break;

		case IOCTL_ACCEL_SET_CONFIG :
			/*
			dprintk ("ioctl: apply arbitrary configuration");

			retval = -EFAULT;
			if (1) {
				accel_config_t config;
				retval = copy_from_user (&config, (int __user *)arg, sizeof(config));
				if (! retval) {
					down (&priv->sem);
					priv->events_mask = ACCEL_EV_ALLBITS;
					up (&priv->sem);

					accel_i2c_write_register (ACCEL_I2C_PORT_INT2_CFG, config.cfg);
					accel_i2c_write_register (ACCEL_I2C_PORT_INT2_THS, config.ths);
					accel_i2c_write_register (ACCEL_I2C_PORT_INT2_DUR, config.dur);

					accel_i2c_write_register (ACCEL_I2C_PORT_REG5, config.reg5);
					accel_i2c_write_register (ACCEL_I2C_PORT_REG4, config.reg4);
					accel_i2c_write_register (ACCEL_I2C_PORT_REG3, config.reg3);
					accel_i2c_write_register (ACCEL_I2C_PORT_REG2, config.reg2);
					accel_i2c_write_register (ACCEL_I2C_PORT_REG1, config.reg1);
				}
			}*/
			retval = -EFAULT;
			break;

		case IOCTL_ACCEL_SET_IRQ_CFG :
		
			dprintk ("ioctl: apply arbitrary configuration");

			retval = -EFAULT;
			if (1) {
				accel_irq_cfg_t config;
				retval = copy_from_user (&config, (int __user *)arg, sizeof(config));
				if (! retval) {
					if (config.orientation == 'P' ||
					    config.orientation == 'p') {
						mutex_lock (&pl);
						portrait[0] = (u8)config.threshold;
						portrait[1] = (u8)config.duration;
						mutex_unlock (&pl);
						dprintk ("IRQ1: ths: 0x%02x dur: 0x%02x", portrait[0], portrait[1]);
					} else
					if (config.orientation == 'L' ||
					    config.orientation == 'l') {
						mutex_lock (&pl);
						landscape[0] = (u8)config.threshold;
						landscape[1] = (u8)config.duration;
						mutex_unlock (&pl);
						dprintk ("IRQ2: ths: 0x%02x dur: 0x%02x", landscape[0], landscape[1]);
					}
					retval = SUCCESS;
				}
			}
			break;
	}
	return (int)retval;
}

/**
 * Poll device
 *
 * @param filp
 * @param wait
 * @return mask
 */
static unsigned int accel_poll(struct file *filp, poll_table *wait)
{
	accel_file_private_t *priv = filp->private_data;
	unsigned int mask = 0;

	down (&priv->sem);
	if (priv->status & ACCEL_ST_DATA_READY) mask |= POLLIN | POLLRDNORM;
	poll_wait (filp, &priv->readq, wait);
	up (&priv->sem);
	return mask;
}


/**
 * Fasync device
 *
 * @param fd
 * @param filp
 * @param mode
 * @return 0 if success, or negative error code 
 */
static int accel_fasync(int fd, struct file *filp, int mode)
{
	accel_file_private_t *priv = filp->private_data;
	return fasync_helper(fd, filp, mode, &priv->asyncq);
}


/**
 * Release device
 *
 * @param inode
 * @param filp
 * @return 0
 */
static int accel_release(struct inode *inode, struct file *filp)
{
	accel_file_private_t *priv = filp->private_data;
	//unsigned long mask;
	pid_t mypid;

	if (priv == NULL)
		return -EBADF;

	mypid = priv->mypid;

	/* remove this filp from the asynchronously notified filp's */
	accel_fasync (-1, filp, 0);

	/* TODO: do we need specific mask here or just clear all since client is just exiting? */
	accel_update_private_mask (priv, 0);

	mutex_lock (&accel_info.mlock);
	
	accel_remove_file_private (priv);	
	kfree (priv);

	if (priv->access_flags & ACCEL_FF_IS_CLIENT) {
		accel_remove_client (mypid);
	}

	/* this sets to NULL client->file_private pointer automatically */
	filp->private_data = NULL;

	mutex_unlock (&accel_info.mlock);

	return 0;
}


struct file_operations accel_fops = {
	.owner   = THIS_MODULE,
	.llseek  = no_llseek,
	.read    = accel_read,
	.write   = accel_write,
	.ioctl   = accel_ioctl,
	.open    = accel_open,
	.poll	 = accel_poll,
	.fasync  = accel_fasync,
	.release = accel_release,
};

