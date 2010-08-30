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

#include "kionix-i2c.h"

/**
 * Sets/updates 8bit register to/with value
 *
 * @param reg Register to be written
 * @param mask Mask
 * @param newval New value
 * @return 0 in success, or negative error code
 */
int accel_i2c_update_register(u8 reg, u8 mask, u8 newval)
{
	u8 val=0;

	if (mask) {
		/* register's value has to be updated */
		val = (u8) i2c_smbus_read_byte_data (&accel_info.i2c, reg);
		//dprintk ("Reading old value 0x%02x from reg 0x%02x", val, reg);
		printk ("Reading old value 0x%02x from reg 0x%02x", val, reg);

		if ((val & mask) & newval) /* given value is already setup */
			return 0;

		val &= ~mask;		
		//dprintk ("Masked 0x%02x value 0x%02x", ~mask, val);
		printk ("Masked 0x%02x value 0x%02x", ~mask, val);
	}

	val |= newval;
	//dprintk ("New value 0x%02x in reg 0x%02x", val, reg);
	printk ("New value 0x%02x in reg 0x%02x", val, reg);

	return i2c_smbus_write_byte_data (&accel_info.i2c, reg, val);
}


/**
 * Reads configuration registers
 *
 * @param reg Array register's content to be written
 * @param number Number of elements in array
 * @return 0 in success, or negative error code
 */
int accel_i2c_read_control_registers(u8 reg[], int number)
{
	if (number < 7) return -ENOMEM;

	reg[0] = SUB_AUTO_INCREMENTED (ACCEL_I2C_PORT_FF_INT);

	/* read registers FF_INT-CTRL_REGA] */
	i2c_master_send (&accel_info.i2c, reg, 1);
	i2c_master_recv (&accel_info.i2c, reg, 7);

	return 0;
}

/**
 * Sets accelerometer power mode
 *
 * @param mode power mode
 * @return 0 in success, or negative error code
 */
int accel_i2c_set_power_mode(int mode)
{
	u8 val;
	if (mode != 0 || mode != 1) return -EFAULT;
	val = 0x80 + (u8)(mode << 6);
	return accel_i2c_update_register (ACCEL_I2C_PORT_REGB, ACCEL_BM_PWR, val);
}


/**
 * Applies basic accelerometer configuration
 *
 * @return 0 in success, or negative error code
 */
void accel_i2c_init(void)
{
	i2c_smbus_write_byte_data (&accel_info.i2c, ACCEL_I2C_PORT_REGB, 0x00);
	i2c_smbus_write_byte_data (&accel_info.i2c, ACCEL_I2C_PORT_REGC, 0x00);
}


/**
 * Sets accelerometer self-test mode
 *
 * @param mode self-test mode
 * @return 0 in success, or negative error code
 */
int accel_i2c_self_test(int mode)
{
/*	u8 val;
	if (mode < PWR_OFF || mode > PWR_tenHZ) return -EFAULT;
	val = (u8)(mode << 1);
	return accel_i2c_update_register (ACCEL_I2C_PORT_REG4, ACCEL_BM_SELF, val);
*/
	return -1;
}


/**
 * Gets accelerometer power mode
 *
 * @return mode in success, or negative error code
 */
int accel_i2c_get_power_mode(int *dest)
{
	int result;

	result = i2c_smbus_read_byte_data (&accel_info.i2c, ACCEL_I2C_PORT_REGB);
	*dest = (result & 0x40) >> 6;

	return *dest;
}

/**
 * Reads WHO_AM_I register
 *
 * @return mode in success, or negative error code
 */
int accel_i2c_who_am_i(void)
{
	return i2c_smbus_read_byte_data (&accel_info.i2c, ACCEL_I2C_PORT_WHO_AM_I);
}

/**
 * Enables X,Y,Z axises
 *
 * @param axis axis to enable
 * @return 0 in success, or negative error code
 */
int accel_i2c_enable_axis(int axis)
{
/*	switch (axis) {
	 case 0 : 
	 case AXIS_X :
	 case AXIS_Y :
	 case AXIS_Z :
	 case AXIS_X | AXIS_Y :
	 case AXIS_X | AXIS_Z :
	 case AXIS_Y | AXIS_Z :
	 case AXIS_XYZ : break;
	 default : return -EFAULT;
	}

	return accel_i2c_update_register (ACCEL_I2C_PORT_REG1, ACCEL_BM_READY_XYZ, (u8)axis);
*/
	return 0;
}


/**
 * Reads 8bit register from chip
 *
 * @param reg Register to be read
 * @param dest Returned value
 * @return 0 in success, or negative error code
 */
int accel_i2c_read_register(u8 reg, int *dest)
{
	*dest = i2c_smbus_read_byte_data (&accel_info.i2c, reg);

	return 0;
}


/**
 * Arms interrupt on chip
 *
 *
 * @param irq user interrupt request
 * @return 0 in success, or negative error code
*/
int accel_i2c_arm_interrupt(int irq, u8 latch, u8 ths, u8 dur, u8 ctrl_reg3)
{
	u8 reg[3];

	switch (irq) {
	 case ACCEL_IRQ_INT1 :
			latch <<= 4;
			reg[0] = SUB_AUTO_INCREMENTED (ACCEL_I2C_PORT_FF_INT); 
				break;
	 case ACCEL_IRQ_INT2 : 
			latch <<= 3;
			reg[0] = SUB_AUTO_INCREMENTED (ACCEL_I2C_PORT_MOT_INT); 
				break;
	 default : return -EFAULT;
	}

	accel_i2c_update_register (ACCEL_I2C_PORT_REGB, ACCEL_BM_INTS, ctrl_reg3);

	/* write to CTRL_REGC register */
	i2c_smbus_write_byte_data (&accel_info.i2c, ACCEL_I2C_PORT_REGC, latch);
	
	reg[1] = ths;
	reg[2] = dur;
	/* write to INT and DELAY registers at once */
	i2c_master_send (&accel_info.i2c, reg, 3);

	return 0;
}


/**
 * Disarms interrupt on chip
 *
 *
 * @param irq irq
 * @return 0 in success, or negative error code
*/
int accel_i2c_disarm_interrupt(int irq)
{
	u8 reg[3]={0,0,0};

	switch (irq) {
	 case ACCEL_IRQ_INT1 :
			reg[0] = SUB_AUTO_INCREMENTED (ACCEL_I2C_PORT_FF_INT); 
				break;
	 case ACCEL_IRQ_INT2 : 
			reg[0] = SUB_AUTO_INCREMENTED (ACCEL_I2C_PORT_MOT_INT); 
				break;
	 default : return -EFAULT;
	}

	accel_i2c_update_register (ACCEL_I2C_PORT_REGB, ACCEL_BM_INTS, 0);

	/* clear CTRL_REGC register */
	i2c_smbus_write_byte_data (&accel_info.i2c, ACCEL_I2C_PORT_REGC, 0);
	
	/* write to INT and DELAY registers at once */
	i2c_master_send (&accel_info.i2c, reg, 3);

	return 0;
}

/**
 * Converts two 8bit into 16bit value
 *
 * @param lsb Lsb byte
 * @param msb Msb byte
 * @return 16bit value
 */
static int accel_axis2val(u8 msb, u8 lsb)
{
	union {
	  u32 uval;
	  int ival;
	} uni;

	uni.uval = ((u32)msb << 4) + (msb >> 4);
	//printk ("lsb=%02x msb=%02x value=%08x(%d)\n", lsb, msb, uni.uval, uni.ival);
	return uni.ival - 2048;
}

static short accel_axis2val_short(u8 msb, u8 lsb, int axis)
{
	union {
	  u16   uval;
	  short sval;
	} uni1, uni2;

	uni1.uval = ((u16)msb << 4) + (lsb >> 4);
	//if (msb & 0x80) uni1.uval |= 0xF000;
	//if (lsb & 0x80) uni2.uval |= 0xF000;
	printk ("%c: lsb=0x%02x msb=0x%02x v1=0x%04x(%d) (%d)\n", 
		axis, lsb, msb, uni1.uval, uni1.sval, uni1.sval - 2048);

	return uni1.sval - 2048;
}

/**
 * Reads axis data from chip
 *
 * @return 0 if success, negative error otherwise
 */
int accel_i2c_read_axis_data(int *axis)
{
	int x, y, z;
	u8 buf[6];

	memset (buf, 0, sizeof(buf));

	buf[0] = SUB_AUTO_INCREMENTED (ACCEL_I2C_PORT_XOUT_H);

	/* read values of 6 consequitive registers starting with XOUT_H */
	i2c_master_send (&accel_info.i2c, buf, 1);
	i2c_master_recv (&accel_info.i2c, buf, 6);

	x = accel_axis2val (buf[0], buf[1]);
	y = accel_axis2val (buf[2], buf[3]);
	z = accel_axis2val (buf[4], buf[5]);
	
	mutex_lock (&accel_info.mlock);

	accel_info.x_axis = x;
	accel_info.y_axis = y;	
	accel_info.z_axis = z;

	mutex_unlock (&accel_info.mlock);

	if (axis) {
		axis[0] = x;
		axis[1] = y;
		axis[2] = z;
	}

	return 0;
}

/**
 * Reads axis data from chip
 *
 * @param axis axis
 * @return 0 if success, negative error otherwise
 */
int accel_i2c_read_axis(int axis)
{
	int val;
	u8 buf[2]={SUB_AUTO_INCREMENTED (ACCEL_I2C_PORT_XOUT_H)};

	memset (buf, 0, sizeof(buf));

	switch (axis) {
	 case AXIS_X : buf[0] = SUB_AUTO_INCREMENTED (ACCEL_I2C_PORT_XOUT_H); break;
	 case AXIS_Y : buf[0] = SUB_AUTO_INCREMENTED (ACCEL_I2C_PORT_YOUT_H); break;
	 case AXIS_Z : buf[0] = SUB_AUTO_INCREMENTED (ACCEL_I2C_PORT_ZOUT_H); break;
	 default: return -EFAULT;
	}

	/* read values of 2 consequitive registers */
	i2c_master_send (&accel_info.i2c, buf, 1);
	i2c_master_recv (&accel_info.i2c, buf, 2);

	val = accel_axis2val (buf[0], buf[1]);

	printk ("[%d]=%d [0x%02x;0x%02x]\n", axis, val, buf[0], buf[1]);

	mutex_lock (&accel_info.mlock);

	switch (axis) {
	 case AXIS_X : accel_info.x_axis = val; break;
	 case AXIS_Y : accel_info.y_axis = val; break;
	 case AXIS_Z : accel_info.z_axis = val; break;
	}

	mutex_unlock (&accel_info.mlock);

	return 0;
}


/**
 * Feed input method
 *
 * Method to support input device
 *
 */
static void accel_feed_input(void)
{
	struct input_dev *idev = accel_info.idev->input;
	int axis[3];

	if (idev) {
		accel_i2c_read_axis_data (axis);

		input_report_abs (idev, ABS_X, axis[0]);
		input_report_abs (idev, ABS_Y, axis[1]);
		input_report_abs (idev, ABS_Z, axis[2]);
		input_sync (idev);
	}
}


/**
 * Irq bottom half handler
 *
 * @param work Bottom half structure
 */
void accel_irq_bottom_half(struct work_struct *work)
{
/*	u8 status, source1, source2;

	status = (u8) i2c_smbus_read_byte_data (&accel_info.i2c, ACCEL_I2C_PORT_REGA);
	printk ("%s: interrupt fired, status 0x%02x", __FUNCTION__, status);

	if (status) {
		dprintk ("interrupt fired");
		status = status;
	}
*/
	accel_feed_input ();
	queue_delayed_work (accel_info.myworkqueue, &accel_info.dw, msecs_to_jiffies(ACCEL_POLL_INTERVAL)); 
}


/**
 * Irq handler
 *
 * Handle IRQ and schedule bottom halves
 *
 * @param irq Irq
 * @param dev_id
 * @return IRQ_HANDLED
 */
irqreturn_t accel_irq_handler(int irq, void *dev)
{
	disable_irq (irq);
	/* we're using latched interrupts */
	schedule_work (&accel_info.wq);
	return IRQ_HANDLED;
}

