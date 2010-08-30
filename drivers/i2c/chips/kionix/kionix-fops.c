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

#include "kionix-main.h"

static void accel_dump_struct (void)
{
	int n;
	ssize_t written = 0;
	char buf[PAGE_SIZE];
	accel_file_private_t p;

	SAFE_SNPRINTF ("\nTotal s: %d \n", sizeof(accel_file_private_t));
	SAFE_SNPRINTF ("o:%2d 'prev'   s:%d\n", offsetof(struct file_private,prev), sizeof(p.prev));
	SAFE_SNPRINTF ("o:%2d 'next'   s:%d\n", offsetof(struct file_private,next), sizeof(p.next));
	SAFE_SNPRINTF ("o:%2d 'mypid'  s:%d\n", offsetof(struct file_private,mypid), sizeof(p.mypid));
	SAFE_SNPRINTF ("o:%2d 'readq'  s:%d\n", offsetof(struct file_private,readq), sizeof(p.readq));
	SAFE_SNPRINTF ("o:%2d 'access' s:%d\n", offsetof(struct file_private,access_flags), sizeof(p.access_flags));
	SAFE_SNPRINTF ("o:%2d 'events' s:%d\n", offsetof(struct file_private,events_mask), sizeof(p.events_mask));
	SAFE_SNPRINTF ("o:%2d 'status' s:%d\n", offsetof(struct file_private,status), sizeof(p.status));
	SAFE_SNPRINTF ("o:%2d 'iev'    s:%d\n", offsetof(struct file_private,iev), sizeof(p.iev));
	SAFE_SNPRINTF ("o:%2d 'nread'  s:%d\n", offsetof(struct file_private,nreaders), sizeof(p.nreaders));
	SAFE_SNPRINTF ("o:%2d 'nwrite' s:%d\n", offsetof(struct file_private,nwriters), sizeof(p.nwriters));
	SAFE_SNPRINTF ("o:%2d 'asyncq' s:%d\n", offsetof(struct file_private,asyncq), sizeof(p.asyncq));
	SAFE_SNPRINTF ("o:%2d 'sem'    s:%d\n", offsetof(struct file_private,sem), sizeof(p.sem));

	fprintk ("%s", buf);
}


static void accel_hex_dump (void *data, int size)
{
	unsigned char *bitt;
	unsigned int *bui;
	int l, k, n;
	ssize_t written = 0;
	char buf[PAGE_SIZE];

	if (data == NULL) return;
	bitt = data;
	SAFE_SNPRINTF ("\n");
	for (l=0;; l++) {
		SAFE_SNPRINTF ("%p: ", bitt);
		for (k=0; k <= 4; k++) {
			bui = (unsigned int *)bitt;
			SAFE_SNPRINTF ("%08x ", *bui);
			bitt += sizeof(unsigned int);
			size -= sizeof(unsigned int);
			if (size == 0) 
				goto print_exit;
		}
		SAFE_SNPRINTF ("\n");
		CUTOFF_CHECK_BREAK (80);
	}
print_exit:
	fprintk ("%s\n", buf);
}

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

	dprintk ("added client pid %d", pid);

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
		prev = next;
		if (next != NULL)
			next->prev = prev;
	}else {
		if (next != NULL)
			next->prev = NULL;
		accel_info.clients = next;
	}
	accel_info.num_clients--;
	kfree (client);

	dprintk ("removed client pid %d", pid);
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

	dprintk ("%d clients notified about event 0x%08lx", done, event);
}

static void accel_update_global_mask (unsigned long bit, int toggle, accel_file_private_t *skip)
{
	accel_file_private_t *priv;
	int found=0;
	
	mutex_lock (&accel_info.mlock);
	priv = accel_info.privates;
	while (priv) {
		if (priv != skip && 
		   (priv->events_mask & bit)) {
			++found;
			break;
		}
		priv = priv->next;
	}

	if (found == 0) {
		if (toggle == TOGGLE_ON)
			accel_events_mask |= bit;
		else	accel_events_mask &= ~bit;

		dprintk ("global events mask 0x%08lx", accel_events_mask);
	}
	mutex_unlock (&accel_info.mlock);
}

static void accel_update_private_mask (accel_file_private_t *priv, unsigned long new)
{
	if (!(new & ACCEL_EV_RAW_DATA)) {

		dprintk ("raw data turned off");

		accel_i2c_set_power_mode (PWR_OFF);
		accel_update_global_mask (ACCEL_EV_RAW_DATA, TOGGLE_OFF, priv);
	}

	if (new & ACCEL_EV_RAW_DATA) {

		dprintk ("raw data turned on");

		accel_i2c_set_power_mode (PWR_ON);
		accel_update_global_mask (ACCEL_EV_RAW_DATA, TOGGLE_ON, priv);
	}
}

static void accel_update_private_mask_orig (accel_file_private_t *priv, unsigned long new)
{
	int i, turned_on, turn_on;
	unsigned long bit_mask, old=priv->events_mask;

	for (i=ACCEL_BITS_IN_ULONG; i >= 0; i--) {
	
		bit_mask = 1 << i;
		if (old & bit_mask)	turned_on = 1;
		else			turned_on = 0;
		if (new & bit_mask)	turn_on = 1;
		else			turn_on = 0;

		if (!(turned_on & turn_on) ||
		     (turned_on && turn_on))
			continue;

		if (turned_on && ! turn_on) { /* Turn Off */

			dprintk ("turn off bit 0x%08lx", bit_mask);

			if (old & ACCEL_EV_SCREEN_ORIENT) {

				dprintk ("screen orientation detection turned off");

				accel_i2c_enable_axis (0);
				accel_i2c_disarm_interrupt (ACCEL_IRQ_INT2);
				accel_i2c_set_power_mode (PWR_halfHZ);
				accel_update_global_mask (ACCEL_EV_SCREEN_ORIENT, TOGGLE_OFF, priv);
			}
		}

		if (! turned_on && turn_on) { /* Turn On */

			dprintk ("turn on bit 0x%08lx", bit_mask);

			if (new & ACCEL_EV_SCREEN_ORIENT) {

				dprintk ("screen orientation detection turned on");

				accel_i2c_enable_axis (AXIS_X | AXIS_Y);
				/*
				 * INT2_CFG=0x4F - 6D, Xh/Xl, Yh/Yl
				 * INT2_THS=0x20 - 500 mg
				 * INT2_DUR=0x03 - 600 ms
				 * CTRL_REG3=0x20 - INT2, LIR2
 				 */
				accel_i2c_arm_interrupt (ACCEL_IRQ_INT2, 0x4F, 0x20, 0x03, 0x20);
				accel_i2c_set_power_mode (PWR_fiveHZ);
				accel_update_global_mask (ACCEL_EV_SCREEN_ORIENT, TOGGLE_ON, priv);
			}
		}
	}
}


/**
 * Resume implementation
 *
 * @param i2c I2C device
 * @return 0
 */
static int accel_resume(struct i2c_client *i2c)
{
	return 0;
}

/**
 * Suspend implementation
 *
 * @param i2c I2C device
 * @param state PM state
 * @return 0
 */
static int accel_suspend(struct i2c_client *i2c, pm_message_t state)
{
	return 0;
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

	dprintk ("open request from pid %d", current->pid);
//accel_dump_struct();

	priv = kzalloc (sizeof(accel_file_private_t), GFP_KERNEL);
	if (priv == NULL) 
		goto err_nomem;

//accel_hex_dump (priv, sizeof(accel_file_private_t));

	client = accel_find_client_by_pid (current->pid);
	if (client == NULL) {
		mutex_lock (&accel_info.mlock);
		client = accel_add_client (current->pid);
		mutex_unlock (&accel_info.mlock);
		if (client == NULL)
			goto err_nomem;
	}

//accel_hex_dump (priv, sizeof(accel_file_private_t));

	/* initialize read wait queue */
	init_waitqueue_head (&priv->readq);
	init_MUTEX (&priv->sem);

//accel_hex_dump (priv, sizeof(accel_file_private_t));

	down (&priv->sem);
	priv->access_flags |= ACCEL_FF_IS_CLIENT;
	priv->access_flags |= ACCEL_FF_IS_VALID;	
	priv->mypid = current->pid;
	up (&priv->sem);

	filp->private_data = priv;

//accel_hex_dump (priv, sizeof(accel_file_private_t));

	/* turn power on every time client comes in */
	rc = i2c_smbus_write_byte_data (&accel_info.i2c, ACCEL_I2C_PORT_REGB, 0xC0);

	mutex_lock (&accel_info.mlock);
	accel_insert_file_private (priv);
	client->file_private = (accel_file_private_t **)&filp->private_data;

	if (rc == 0) accel_info.mode = PWR_ON;

	mutex_unlock (&accel_info.mlock);

	if (rc) {
		dprintk ("Accelerometer power up has failed");
	}else {
		dprintk ("Accelerometer powered up");	
	}

//accel_hex_dump (priv, sizeof(accel_file_private_t));

	return nonseekable_open(inode, filp);

err_nomem:
	rc = -ENOMEM;
	return rc;
}


/**
 * Read implementation
 *
 * Reads the acceleration data (X,Y,Z) and outputs them to user.
 *
 * @param f Kernel level file structure
 * @param buffer Userspace buffer
 * @param bytes Bytes requested
 * @param offset Offset requested
 */
static int accel_read(struct file *f, char __user *buffer, size_t bytes, long long *offset)
{
	char g_values[64];

	if (accel_i2c_read_axis_data (NULL))
		return -EFAULT;

	mutex_lock (&accel_info.mlock);
	snprintf(g_values, sizeof(g_values), "%d:%d:%d\n", 
			accel_info.x_axis,
			accel_info.y_axis, 
			accel_info.z_axis);
	mutex_unlock (&accel_info.mlock);

	if (*offset >= strlen(g_values))
		return 0;

	if (copy_to_user(buffer, g_values, strlen(g_values)))
		return -EFAULT;

	return strlen(g_values);
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
	int err = 0, result;
	int axis[3], retval = 0;
	int i_data = 0;
	unsigned int ui_data;
	unsigned long ul_data, ul_prev;

	if (_IOC_TYPE(cmd) != ACCEL_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > ACCEL_IOC_MAXNR) return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;

	if (! capable (CAP_SYS_ADMIN)) {
		return -EPERM;
	}

	switch (cmd) {
		case IOCTL_ACCEL_EVENTS_SET :

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

		case IOCTL_ACCEL_EVENTS_GET :

			down (&priv->sem);
			ul_data = priv->events_mask;
			up (&priv->sem);

			dprintk ("events mask get 0x%08lx", ul_data);

			retval = copy_to_user ((int __user *)arg, &ul_data, sizeof(ul_data));
			break;

		case IOCTL_ACCEL_EVENTS_CLEAR :

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

		case IOCTL_ACCEL_FETCH_EVENT_DATA :

			dprintk ("fetch event data");

			retval = -EFAULT;
			down (&priv->sem);
			if (priv->status & ACCEL_ST_DATA_READY)
				retval = copy_to_user ((int __user *)arg, &priv->iev, sizeof(priv->iev));
			priv->status &= ~ACCEL_ST_DATA_READY;
			up (&priv->sem);
			break;

		case IOCTL_ACCEL_SELF_TEST:
			retval = -EFAULT;
			break;

		case IOCTL_ACCEL_SET_PWR_MODE:
			retval = -EFAULT;
			break;

		case IOCTL_ACCEL_GET_PWR_MODE:
			retval = -EFAULT;
			break;

		case IOCTL_ACCEL_POLL_DELAY_SET:
			retval = __get_user (ui_data, (unsigned int __user *)arg);
			
			if (! retval ) {
				/* holding mlock will block accel_input_poll() from entering/leaving
				 * it should be enough for input_polled_device_work() to complete
				 */
				mutex_lock (&accel_info.mlock);
				accel_info.idev->poll_interval = ui_data;
				mutex_unlock (&accel_info.mlock);

				retval = SUCCESS;
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
	pid_t mypid;
	int rc;

	if (priv == NULL)
		return -EBADF;

	mypid = priv->mypid;

	/* remove this filp from the asynchronously notified filp's */
	accel_fasync (-1, filp, 0);

	mutex_lock (&accel_info.mlock);
	
	accel_remove_file_private (priv);	
	kfree (priv);

	if (priv->access_flags & ACCEL_FF_IS_CLIENT) {
		accel_remove_client (mypid);
	}

	/* this sets to NULL client->file_private pointer automatically */
	filp->private_data = NULL;

	/* turn power off every time client leaves */
	rc = i2c_smbus_write_byte_data (&accel_info.i2c, ACCEL_I2C_PORT_REGB, 0x00);
	if (rc == 0)	accel_info.mode = PWR_OFF;

	mutex_unlock (&accel_info.mlock);

	if (rc) {
		dprintk ("Accelerometer power down has failed");
	}else {
		dprintk ("Accelerometer powered down");
	}

	return 0;
}


struct file_operations accel_fops = {
	.owner   = THIS_MODULE,
	.llseek  = no_llseek,
	.read    = accel_read,
	.ioctl   = accel_ioctl,
	.open    = accel_open,
	.poll	 = accel_poll,
	.fasync  = accel_fasync,
	.release = accel_release,
};

