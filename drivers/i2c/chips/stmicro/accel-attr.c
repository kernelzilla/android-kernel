/*
 * Morrison accelerometer I2C protocol driver
 *
 *	Copyright (C) 2008  Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License v2 as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */

#include "accel-main.h"
#include "accel-i2c.h"

/* Module attributes stuff */
enum accel_cmd
{
	ACCEL_CMD_CURRENT = 0,
	ACCEL_CMD_REGISTERS,
	ACCEL_CMD_CLIENTS,
	ACCEL_CMD_MODE,
	ACCEL_CMD_MAX
};
/*
static char *orientation_names[] = {"unknown", "portrait", "landscape", "normal", "upside-down", 
				    "tilted_left", "tilted_right" };
*/
/**
 * Attribute Show Current
 * Prints out current X,Y and Z values
 * @return pointer to character string
 */
static ssize_t accel_show_current(struct device *dev, struct device_attribute *attr, char *buf)
{
	int x, y, z; //axes[3]={0}; 
	mutex_lock (&accel_info.mlock);
	//accel_i2c_read_axis_data (axes);
	x = accel_info.x_axis, y = accel_info.y_axis, z = accel_info.z_axis;
	mutex_unlock (&accel_info.mlock);
	return snprintf (buf, PAGE_SIZE, "%d:%d:%d\n", x, y, z);
	//return snprintf (buf, PAGE_SIZE, "%d:%d:%d\n", axes[0], axes[1], axes[2]);
}


/**
 * Attribute Show Registers
 * Prints out current accelerometer's registers content
 * @return pointer to character string
 */
static ssize_t accel_show_registers(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t n, written=0;
	u8 regs[13];

	if (accel_i2c_read_control_registers (regs, sizeof(regs)/sizeof(regs[0])) != 0)
		return -EINVAL;
				
	SAFE_SNPRINTF ("CTRL_REG1 0x%02x\n", regs[0]);
	SAFE_SNPRINTF ("CTRL_REG2 0x%02x\n", regs[1]);
	SAFE_SNPRINTF ("CTRL_REG3 0x%02x\n", regs[2]);
	SAFE_SNPRINTF ("CTRL_REG4 0x%02x\n", regs[3]);
	SAFE_SNPRINTF ("CTRL_REG5 0x%02x\n", regs[4]);
	SAFE_SNPRINTF ("INT1_CFG  0x%02x\n", regs[5]);
	SAFE_SNPRINTF ("INT1_SRC  0x%02x\n", regs[6]);
	SAFE_SNPRINTF ("INT1_THS  0x%02x\n", regs[7]);
	SAFE_SNPRINTF ("INT1_DUR  0x%02x\n", regs[8]);
	SAFE_SNPRINTF ("INT2_CFG  0x%02x\n", regs[9]);
	SAFE_SNPRINTF ("INT2_SRC  0x%02x\n", regs[10]);
	SAFE_SNPRINTF ("INT2_THS  0x%02x\n", regs[11]);
	SAFE_SNPRINTF ("INT2_DUR  0x%02x\n", regs[12]);

	return written;
}


/**
 * Attribute Show Clients
 * Prints out list of registered clients
 * @return pointer to character string
 */
static ssize_t accel_show_clients(struct device *dev, struct device_attribute *attr, char *buf)
{
	accel_client_t *client;
	accel_file_private_t *priv;
	ssize_t n, written=0;

	SAFE_SNPRINTF (" Pid      | Mode     |Event mask| Status |\n");
	SAFE_SNPRINTF ("==========================================\n"); 
	
	mutex_lock (&accel_info.mlock);
	client = accel_info.clients;
	while (client != NULL) {

		priv = accel_find_private_by_pid (client->pid);
		if (priv != NULL) {
			SAFE_SNPRINTF ("%-10d|0x%08lx|0x%08lx|0x%08x|\n",
				priv->mypid, 
				priv->access_flags,
				priv->events_mask,
				priv->status);
		}else {
			SAFE_SNPRINTF ("Oops!!! Private for pid %d not found\n", client->pid);
		}
		client = client->next;
		CUTOFF_CHECK_BREAK (80);	
	}
	mutex_unlock (&accel_info.mlock);

	return written;
}


/**
 * Attribute Show accelerometer driver mode
 * Prints out current mode
 * @return pointer to character string
 */
static ssize_t accel_show_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	int mode, fs, irqs;
	ssize_t len=0;

	accel_i2c_read_register (ACCEL_I2C_PORT_REG1, &mode);
	accel_i2c_read_register (ACCEL_I2C_PORT_REG3, &irqs);
	accel_i2c_read_register (ACCEL_I2C_PORT_REG4, &fs);
	*buf = 0;
	mutex_lock (&accel_info.mlock);
#ifdef ACCEL_DEBUG_IRQ
	snprintf (buf, PAGE_SIZE, "P/Li:%d P/Lw:%d [1]:%d, [2]:%d; D/Ri:%d D/Rw:%d\n",
		accel_info.int1_isr_cnt, accel_info.int1_bh_cnt, 
		accel_info.irq1_cnt, accel_info.irq2_cnt,
		accel_info.int2_isr_cnt, accel_info.int2_bh_cnt);

	accel_info.int1_isr_cnt = accel_info.int1_bh_cnt =
	accel_info.int2_isr_cnt = accel_info.int2_bh_cnt =
	accel_info.irq1_cnt = accel_info.irq2_cnt = 0;
#endif
	if (! accel_events_mask) {
		if (! irqs)	strcat (buf, "ON:");
		else		strcat (buf, "OFF:");
	} else {
		if (accel_events_mask & ACCEL_EV_SCREEN_ORIENT) strcat (buf, "P/L:");
		if (accel_events_mask & ACCEL_EV_RAW_DATA) strcat (buf, "R/D:");
	}
	mutex_unlock (&accel_info.mlock);
	//dprintk ("mode 0x%08x; mode & ODR 0x%02x; mode & PWR 0x%02x\n", mode, mode & ACCEL_BM_ODR, mode & ACCEL_BM_PWR);
	if ((mode & ACCEL_BM_ODR) == 0x08) mode = 100;
	else if ((mode & ACCEL_BM_ODR) == 0x10) mode = 400;
	else if ((mode & ACCEL_BM_PWR) == 0x20) mode = 50;
	else if ((mode & ACCEL_BM_PWR) == 0xC0) mode = 10;
	else if ((mode & ACCEL_BM_PWR) == 0x00) mode = 0;
	if ((fs & ACCEL_BM_SCALE) == 0x10) fs=4;
	else if ((fs & ACCEL_BM_SCALE) == 0x00) fs=2;
	else fs=-1;
	len = strlen (buf);
#ifndef ACCEL_DATA_READY_IRQ
	return len + snprintf (buf+len, PAGE_SIZE-len, "%dHz:%dms:%dg\n", 
        mode, accel_info.ipdev->poll_interval, fs);
#else
	return len + snprintf (buf+len, PAGE_SIZE-len, "%dHz:%dg\n", 
        mode, fs);
#endif
}
#ifndef CONFIG_MACH_MOT
static char *accel_print_args(int arg_num, char *args[])
{
	static char buffer[32];
	volatile int cp=0, to=30;
	int i, n;	
	n = snprintf (buffer, to, "arg#=%d,", arg_num);
	cp += n;
	to -= n;
	for (i=0; i < arg_num; i++) {
		n = snprintf (buffer+cp, to, " [%d]=%s", i, args[i]);
		cp += n;
		to -= n;
	}
	return buffer;
}
#endif
#include <linux/ctype.h>

static int convert(unsigned char symbol) {
    int result = symbol - 0x30;
    if (result > 0x30) return result - 39;
    if (result > 0x10) return result - 7;
    return result;
}

u8 _regs[] = { ACCEL_I2C_PORT_REG1, ACCEL_I2C_PORT_REG2, ACCEL_I2C_PORT_REG3, ACCEL_I2C_PORT_REG4,
               ACCEL_I2C_PORT_INT1_CFG, ACCEL_I2C_PORT_INT1_THS, ACCEL_I2C_PORT_INT1_DUR,
               ACCEL_I2C_PORT_INT2_CFG, ACCEL_I2C_PORT_INT2_THS, ACCEL_I2C_PORT_INT2_DUR };

static ssize_t accel_set_registers(struct device *dev, struct device_attribute *attr, const char *in, size_t count) {
    unsigned char buf[10];
    int i = 0;
    int j = 2; // offset for first 2 bytes which are the flags for accel_events_mask
    int regs = 10;

    fprintk ("not supported");
    return -EINVAL;

    if (count < (2 * regs + 2)) { // all regs + 2 bytes for flags
        return 0;
    }
    for (; i < regs; i++) {
        buf[i] = convert(in[j++]) << 4;
        buf[i] += convert(in[j++]);
    }
/* It can happens before chip get out of suspend.  Wake it up here*/
	mutex_lock (&accel_info.mlock);
	if (accel_info.suspended) {
		mutex_unlock (&accel_info.mlock);
		accel_resume (NULL);	
		dprintk ("driver forced off suspend");
	} else {
		mutex_unlock (&accel_info.mlock);
	}
    accel_i2c_write_register(ACCEL_I2C_PORT_REG1, 0x07); // turn it off
 	mutex_lock (&accel_info.mlock);
    if (in[0] == '1') { // DATA_READY
        accel_events_mask |= ACCEL_EV_RAW_DATA;
    } else {
        accel_events_mask &= ~ACCEL_EV_RAW_DATA;
    }
    if (in[1] == '1') { // ORIENT
        accel_events_mask |= ACCEL_EV_SCREEN_ORIENT;
    } else {
        accel_events_mask &= ~ACCEL_EV_SCREEN_ORIENT;
    }
    mutex_unlock(&accel_info.mlock);
    for (i = 0; i < regs; i++) {
        if (buf[i] != 0xff) {
            accel_i2c_write_register(_regs[i], buf[i]);
        }
    }
    return 9;
}

/**
 * Attribute Set accelerometer driver mode
 * Sets given mode
 * @return pointer to character string
 */
static ssize_t accel_set_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int mode = -1;
    int fs = 0;
    int odr = -1;
    const char *s, *e, *buf_end;
    
    if (!memcmp(buf, "raw", 3)) { // special handling
        return accel_set_registers(dev, attr, buf+3, count - 3);
    }

    buf_end = buf + count;
    s = buf;
    while (s < buf_end) {
	while ((s < buf_end) && (*s == ' '))
	    s++;
	e = s;
	while ((e < buf_end) && isalnum (*e))
	    e++;

	if (e == s) break;

	if (e == s + 1) { // mode or ODR
	    switch (*s) {
		case '0': mode = 0; break; // off
		case '1': mode = 1; break; // D/R
		case '2': mode = 2; break; // P/L
		case '3': mode = 3; break; // D/R+P/L
		case '4': mode = 4; break; // pure power on
		case '5': mode = 5; break; // Hong's setup
		case '9': mode = 9; break; // resend P/L event
		case 's': case 'S': odr = 0; break; // slow
		case 'm': case 'M': odr = 1; break; // medium
		case 'f': case 'F': odr = 2; break; // fast
		case 'h': case 'H': odr = 3; break; // hyper
	    }
	} else if (*s == '4') { // FS comes as '4g'
		fs = 1;
	}

	s = e;
    }

    if (((mode == -1) && (odr != -1)) || ((mode != 0) && (odr == -1))) {
        fprintk("invalid mode or ODR");
        return count;
    }
    if( mode == 9 )  /* just send current state */
    {
      if( accel_events_mask & ACCEL_EV_SCREEN_ORIENT )       
          accel_irq_bottom_half_job(1);
     return count;
    }
  
    vprintk ("%s; mode %d, ODR %d, FS %d", buf, mode, odr, fs);
    accel_i2c_set_config (mode, odr, fs, -1);
    return count;	
}


static SENSOR_DEVICE_ATTR(show_current, S_IRUGO, 
		accel_show_current, NULL, ACCEL_CMD_CURRENT);
static SENSOR_DEVICE_ATTR(show_registers, S_IRUGO | S_IWUSR, 
		accel_show_registers, accel_set_registers, ACCEL_CMD_REGISTERS);
static SENSOR_DEVICE_ATTR(show_clients, S_IRUGO, 
		accel_show_clients, NULL, ACCEL_CMD_CLIENTS);
static SENSOR_DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, 
		accel_show_mode, accel_set_mode, ACCEL_CMD_MODE);
static SENSOR_DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, 
		accel_show_registers, accel_set_registers, ACCEL_CMD_REGISTERS);


static struct attribute *accel_attributes[] = {
	&sensor_dev_attr_show_current.dev_attr.attr,
	&sensor_dev_attr_show_registers.dev_attr.attr,
	&sensor_dev_attr_show_clients.dev_attr.attr,
	&sensor_dev_attr_mode.dev_attr.attr,
	&sensor_dev_attr_registers.dev_attr.attr,
	NULL
};

struct attribute_group accel_defattr_group = {
	.attrs = accel_attributes,
};

