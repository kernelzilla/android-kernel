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

#include "accel-i2c.h"

#define KEYWORD(symbol,tag,odr,multi)		odr,
static int ODR_table [ACCEL_PWR_MAX] = {ACCEL_POWER_DEFS};
#undef KEYWORD

#define KEYWORD(symbol,tag,odr,multi)		multi,
static int MULTI_table [ACCEL_PWR_MAX] = {ACCEL_POWER_DEFS};
#undef KEYWORD
static void accel_handle_data_ready (void);

/*
 * Orientation calculation block uses the following parameters when orientation interrupt is not used
 */
static int	x_minus = 0,
		x_plus = 0,
		y_minus = 0,
		y_plus = 0,
		z_minus = 0,
		z_plus = 0;

static int 	accel_average_num = 0;

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
		val = (u8) i2c_smbus_read_byte_data (accel_info.i2c, reg);
		//dprintk ("Reading old value 0x%02x from reg 0x%02x", val, reg);

		if ((val & mask) & newval) /* given value is already setup */
			return 0;

		val &= ~mask;		
		//dprintk ("Masked 0x%02x value 0x%02x", (~mask) & 0xFF, val);
	}

	val |= newval;
	dprintk ("new value 0x%02x in reg 0x%02x", val, reg);

	return i2c_smbus_write_byte_data (accel_info.i2c, reg, val);
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
	if (number < 13) return -ENOMEM;

	reg[0] = SUB_AUTO_INCREMENTED (ACCEL_I2C_PORT_REG1);

	/* read CTRL registers [0-4] */
	i2c_master_send (accel_info.i2c, reg, 1);
	i2c_master_recv (accel_info.i2c, reg, 5);

	/* read interrupt configuration */
	reg[5] = SUB_AUTO_INCREMENTED (ACCEL_I2C_PORT_INT1_CFG);
	i2c_master_send (accel_info.i2c, reg+5, 1);
	i2c_master_recv (accel_info.i2c, reg+5, 8);

	return 0;
}

/**
 * Gets accelerometer power mode
 *
 * @return mode in success, or negative error code
 */
int accel_i2c_get_power_mode(int *dest)
{
	int result;

	result = i2c_smbus_read_byte_data (accel_info.i2c, ACCEL_I2C_PORT_REG1);
	*dest = (result & 0xFF) >> 5;

	return *dest;
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
	int effective_mode;
	if (mode < PWR_OFF || mode > PWR_tenHZ) return -EFAULT;
	val = (u8)(mode << 5);

	accel_i2c_get_power_mode (&effective_mode);
	//dprintk ("mode %d, effective %d", mode, effective_mode);
	if (mode != effective_mode) { 

		//accel_i2c_update_register (ACCEL_I2C_PORT_REG1, ACCEL_BM_PWR, 0);
		accel_i2c_update_register (ACCEL_I2C_PORT_REG1, ACCEL_BM_PWR, val);
		accel_i2c_get_power_mode (&effective_mode);
	}
	/*
	 * Since we watch effective ODR, we need make sure power mode was applied 
	 */
	if (mode == effective_mode) {

		//mutex_lock (&accel_info.mlock);

		accel_info.ODR = ODR_table [effective_mode];
		accel_info.powermode = effective_mode;

		/* change orientation calculation block parameters */
		accel_average_num = MULTI_table [effective_mode];
		x_minus = 0; x_plus = 0; y_minus = 0; y_plus = 0; z_minus = 0; z_plus = 0;		

		//mutex_unlock (&accel_info.mlock);
	}
	return 0;
}

/**
 * Sets accelerometer self-test mode
 *
 * @param mode self-test mode
 * @return 0 in success, or negative error code
 */
int accel_i2c_self_test(int mode)
{
	u8 val;
	if (mode < PWR_OFF || mode > PWR_tenHZ) return -EFAULT;
	val = (u8)(mode << 1);
	return accel_i2c_update_register (ACCEL_I2C_PORT_REG4, ACCEL_BM_SELF, val);
}

/**
 * Reads WHO_AM_I register
 *
 * @return mode in success, or negative error code
 */
int accel_i2c_who_am_i(void)
{
	return i2c_smbus_read_byte_data (accel_info.i2c, ACCEL_I2C_PORT_WHO_AM_I);
}

/**
 * Enables X,Y,Z axises
 *
 * @param axis axis to enable
 * @return 0 in success, or negative error code
 */
int accel_i2c_enable_axis(int axis)
{
	switch (axis) {
	 case 0 : /* disable */
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
	*dest = i2c_smbus_read_byte_data (accel_info.i2c, reg);

	return 0;
}


/**
 * Writes value to 8bit register
 *
 * @param reg Register to be written to
 * @param val Value to write
 * @return 0 in success, or negative error code
 */
int accel_i2c_write_register(u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data (accel_info.i2c, reg, val);
}

// Mutex and arrays
//struct mutex pl;		/* data modification lock */
//u8	portrait[2]={0}, landscape[2]={0};

static struct mode_cfg all_modes[]={ 
	// 2G and 4G full scales
	// CVK011C adjusted debounce time
	{0xc7, 0xc0, 0x86, 0x25, 0x05, 0x89, 0x2a, 0x05}, // 10Hz
	{0x27, 0xc0, 0x86, 0x25, 0x19, 0x89, 0x2a, 0x19}, // 50Hz
	{0x2f, 0xc0, 0x86, 0x25, 0x32, 0x89, 0x2a, 0x32}, //100Hz
	{0x37, 0xc0, 0x86, 0x25, 0xc8, 0x89, 0x2a, 0xc8}, //400Hz

	{0xc7, 0xd0, 0x86, 0x05, 0x05, 0x89, 0x05, 0x05}, // 10Hz
	{0x27, 0xd0, 0x86, 0x05, 0x19, 0x89, 0x05, 0x19}, // 50Hz
	{0x2f, 0xd0, 0x86, 0x05, 0x32, 0x89, 0x05, 0x32}, //100Hz
	{0x37, 0xd0, 0x86, 0x05, 0xc8, 0x89, 0x05, 0xc8}  //400Hz
};

#define SEC 1000  // Msec in sec - our clock is screwed up
unsigned odr2interval (int odr)
{
    switch (odr) {
        case 3: 
            return SEC/400; break;
        case 2: 
            return SEC/100; break;
        case 1: 
            return SEC/50; break;
        case 0: 
        default: 
            return SEC/10; break;
    }
    return SEC/10;
}

static char *accel_mode2name (int mode)
{
    switch (mode) {
        case (0): return "power off"; break;
        case (1): return "raw data"; break;
        case (2): return "p/l"; break;
        case (3): return "raw data + p/l"; break;
        case (4): return "power on, no irq"; break;
        default: return "unknown";
    }
}

static char *accel_odr2name (int odr)
{
    switch (odr) {
        case (3): return "400Hz"; break;
        case (2): return "100Hz"; break;
        case (1): return "50Hz"; break;
        case (0): 
        default:
            return "10Hz"; break;
    }
}

int accel_i2c_set_config(int mode, int odr, int fs)
{
	//mutex_lock (&accel_info.mlock);
	if (accel_info.suspended) {
		//mutex_unlock (&accel_info.mlock);
		accel_resume (NULL);	
		dprintk ("driver forced off suspend");
	} else {
		//mutex_unlock (&accel_info.mlock);
	}

    fprintk ("mode=%d (%s), odr=%d (%s), fs=%d\n", 
        mode, accel_mode2name (mode), odr, accel_odr2name (odr), fs);
    // power off
	i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_REG1, 0x07); 

	if (0 == mode || 4 == mode) {
		// power off - 0, or power on w/no interrupts - 4
		//mutex_lock (&accel_info.mlock);
		accel_events_mask &= ~ACCEL_EV_SCREEN_ORIENT;
		accel_events_mask &= ~ACCEL_EV_RAW_DATA;

        atomic_set (&accel_info.data_ready_enabled, 0);
        accel_info.ipdev->poll_interval = ACCEL_NO_POLL;
		if (accel_info.idev) {
			input_report_switch (accel_info.idev, 6, TOGGLE_OFF);
			input_sync (accel_info.idev);			
		}
		//mutex_unlock (&accel_info.mlock);

		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_INT1_CFG, 0x00); // INT1 off
		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_INT2_CFG, 0x00); // INT2 off

		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_INT1_THS, 0x00); // INT1 clean up 
		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_INT2_THS, 0x00); // INT2 clean up

		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_INT1_DUR, 0x00); // INT1 clean up
		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_INT2_DUR, 0x00); // INT2 clean up

        i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_REG3, 
            ACCEL_RAW_DATA_OFF | ACCEL_PORT_LAND_OFF);	// R/D=off P/L=off value 0x1B

		if (4 == mode) {
			struct mode_cfg *config = all_modes+odr+fs*4;

			i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_REG1, 
                config->ctrl_reg1);
			i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_REG4, 
                config->ctrl_reg4);
		}

        // cancel next work queue
    	cancel_delayed_work_sync(&accel_info.wq1);
	} else
	  if (1 == mode) {
		// raw data
		struct mode_cfg *config = all_modes+odr+fs*4;

		//mutex_lock (&accel_info.mlock);
		accel_events_mask |= ACCEL_EV_RAW_DATA;
		accel_events_mask &= ~ACCEL_EV_SCREEN_ORIENT;

		if (accel_info.idev) {
			input_report_switch (accel_info.idev, 6, TOGGLE_OFF);			
			input_sync (accel_info.idev);			
		}

        atomic_set (&accel_info.data_ready_enabled, 1);
        if (accel_info.ipdev->poll_interval == ACCEL_NO_POLL) {
            input_stop_polled_device (accel_info.ipdev);
            accel_info.ipdev->poll_interval = odr2interval (odr);
            input_start_polled_device (accel_info.ipdev);
        } else {
            accel_info.ipdev->poll_interval = odr2interval (odr);
        }

        fprintk ("setting poll_interval to %dms", 
            accel_info.ipdev->poll_interval);
		//mutex_unlock (&accel_info.mlock);

		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_INT1_CFG, 
            0x00); // INT1 off
		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_INT2_CFG, 
            0x00); // INT2 off

        i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_REG3, 
            ACCEL_RAW_DATA_OFF | ACCEL_PORT_LAND_OFF);	// P/L=off
		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_REG1, 
            config->ctrl_reg1);
		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_REG4, 
            config->ctrl_reg4);

        // cancel next work queue
    	cancel_delayed_work_sync(&accel_info.wq1);
	} else
	  if (2 == mode) {
		// P/L interrupt based orientation
		struct mode_cfg *config = all_modes+odr+fs*4;

		//mutex_lock (&accel_info.mlock);
		accel_events_mask |= ACCEL_EV_SCREEN_ORIENT;
		accel_events_mask &= ~ACCEL_EV_RAW_DATA;

		if (accel_info.idev) {
			input_report_switch (accel_info.idev, 6, TOGGLE_ON);			
			input_sync (accel_info.idev);			
		}

        atomic_set (&accel_info.data_ready_enabled, 0);
        accel_info.ipdev->poll_interval = ACCEL_NO_POLL;

		//mutex_unlock (&accel_info.mlock);

		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_INT1_CFG, 
            0x00); // INT1 off
		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_INT2_CFG, 
            0x00); // INT2 off

		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_REG3, 
			ACCEL_RAW_DATA_OFF | ACCEL_PORT_LAND_ON);	// R/D=off P/L=on value 0x19
		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_REG1, 
            config->ctrl_reg1);
		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_REG4, 
            config->ctrl_reg4);

        // schedule next work queue
		schedule_delayed_work(&accel_info.wq1, msecs_to_jiffies(ACCEL_ORIENT_POLL_INTERVAL));
	} else
	  if (3 == mode) {
		 // 2+3 together
		struct mode_cfg *config = all_modes+odr+fs*4;

		//mutex_lock (&accel_info.mlock);
		accel_events_mask |= ACCEL_EV_RAW_DATA;
		accel_events_mask |= ACCEL_EV_SCREEN_ORIENT;

		if (accel_info.idev) {
			input_report_switch (accel_info.idev, 6, TOGGLE_ON);			
			input_sync (accel_info.idev);			
		}

        atomic_set (&accel_info.data_ready_enabled, 1);
        if (accel_info.ipdev->poll_interval == ACCEL_NO_POLL) {
            input_stop_polled_device (accel_info.ipdev);
            accel_info.ipdev->poll_interval = odr2interval (odr);
            input_start_polled_device (accel_info.ipdev);
        } else {
            accel_info.ipdev->poll_interval = odr2interval (odr);
        }

        fprintk ("setting poll_interval to %dms", 
            accel_info.ipdev->poll_interval);
		//mutex_unlock (&accel_info.mlock);

		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_INT1_CFG, 
            0x00); // INT1 off
		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_INT2_CFG, 
            0x00); // INT2 off

        // R/D=on P/L=on value 0x11
		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_REG3, 
			ACCEL_RAW_DATA_OFF | ACCEL_PORT_LAND_ON);	 
		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_REG1, 
            config->ctrl_reg1);
		i2c_smbus_write_byte_data (accel_info.i2c, ACCEL_I2C_PORT_REG4, 
            config->ctrl_reg4);

        // schedule next work queue
		schedule_delayed_work(&accel_info.wq1, msecs_to_jiffies(ACCEL_ORIENT_POLL_INTERVAL));
	}

	return 0;
}


/**
 * Converts two 8bit into 32bit value
 *
 * @param lsb Lsb byte
 * @param msb Msb byte
 * @return 32bit value
 */
static int accel_axis2val(u8 lsb, u8 msb)
{
    u16 uval;

	uval = ((u16)lsb << 8) + (msb & 0xF0);
    //printk_data ("0x%x 0x%x, uval = 0x%x", lsb, msb, uval);
    
    return ((s16)uval)>>4;
}

/**
 * Reads axis data from chip
 *
 * @return 0 if success, negative error otherwise
 */
int accel_i2c_read_xyz(int *axis)
{
	int x, y, z;
    int ret, ret1;
    int xx = 1, yy = 3, zz = 5;
	u8 buf[7]={SUB_AUTO_INCREMENTED (ACCEL_I2C_PORT_STATUS)};

	if (axis) {
		axis[0] = accel_info.x_axis;
		axis[1] = accel_info.y_axis;
		axis[2] = accel_info.z_axis;
	}
	/* read values of 6 consequitive registers starting with XOUT_L */
	//mutex_lock (&accel_info.mlock);
	ret = i2c_master_send (accel_info.i2c, buf, 1);
    if (ret < 0) {
        //mutex_unlock (&accel_info.mlock);
        fprintk ("i2c_master_send error %d", ret);
        return ret;
    }
	ret1 = i2c_master_recv (accel_info.i2c, buf, 7);
    if (ret1 < 0) {
        //mutex_unlock (&accel_info.mlock);
        fprintk ("i2c_master_recv error %d", ret1);
        return ret;
    }
    //mutex_unlock (&accel_info.mlock);

    if ((buf[0] & 0x0F) == 0) {
        // No data available
        printk_bad_data ("no data available: 0x%02X", buf[0]);
        return -ENOMEM;
    }
    x = accel_info.x_axis;
    y = accel_info.y_axis;
    z = accel_info.z_axis;
	/*
	 * Swap axis' values if accelerometer's coordinate system does not match 
	 *  default device's coordinate system
	 */
	switch (accel_param_swapped) {
	 case SWAPPED_YZ : 
         if (buf[0] & 0x9) {
            x = accel_axis2val (buf[1], buf[2]);
         }
         if (buf[0] & 0xA) {
            z = accel_axis2val (buf[3], buf[4]);
         }
         if (buf[0] & 0xB) {
            y = accel_axis2val (buf[5], buf[6]); 
         }
         zz = 3;
         yy = 5;
		 break;
	 case SWAPPED_XZ :		
         if (buf[0] & 0x9) {
		     z = accel_axis2val (buf[1], buf[2]);
         }
         if (buf[0] & 0xA) {
             y = accel_axis2val (buf[3], buf[4]);
         }
         if (buf[0] & 0xB) {
             x = accel_axis2val (buf[5], buf[6]); 
         }
         zz = 1;
         xx = 5;
         break;
	 case SWAPPED_XY :
		// Motus specific axes layout		
         if (buf[0] & 0x9) {
             y = accel_axis2val (buf[1], buf[2]);
         }
         if (buf[0] & 0xA) {
             x = -accel_axis2val (buf[3], buf[4]);
         }
         if (buf[0] & 0xB) {
             z = accel_axis2val (buf[5], buf[6]); 
         }
         yy = 1;
         xx = 3;
		 break;
	 default:
		// Morrison specific axes layout
         if (buf[0] & 0x9) {
             x = -accel_axis2val (buf[1], buf[2]);
         }
         if (buf[0] & 0xA) {
             y = -accel_axis2val (buf[3], buf[4]);
         }
         if (buf[0] & 0xB) {
             z = accel_axis2val (buf[5], buf[6]); 
         }
	}	
    if (x == 0 && y == 0 && z == 0) {
        printk_bad_data ("bad data: all coordinates are 0");
        return -EINVAL;
    }

    printk_data ("%02X; %02X%02X x=%3d, %02X%02X y=%3d, %02X%02X z=%3d",
        buf[0], buf[xx+1], buf[xx], x, buf[yy+1], buf[yy], y, 
        buf[zz+1], buf[zz], z);

    if((buf[0] == buf[1]) && (buf[0] == buf[2]) && (buf[0] == buf[3]) && 
       (buf[4] == buf[5]) && (buf[4] == buf[6]))
    {
        printk_bad_data ("bad data: garbage...");
        return -EINVAL;
    }

	//mutex_lock (&accel_info.mlock);
	accel_info.x_axis = x;
	accel_info.y_axis = y;	
	accel_info.z_axis = z; 
	//mutex_unlock (&accel_info.mlock);

	if (axis) {
		axis[0] = x;
		axis[1] = y;
		axis[2] = z;
	}

	return 0;
}

static char *accel_orient2name (int orient)
{
    switch (orient) {
        case (ACCEL_ORIENT_NORMAL): return "normal"; break;
        case (ACCEL_ORIENT_FACE_UP): return "face up"; break;
        case (ACCEL_ORIENT_FACE_DOWN): return "face down"; break;
        case (ACCEL_ORIENT_UPSIDE): return "upside"; break;
        case (ACCEL_ORIENT_TILTED_LEFT): return "tilted left"; break;
        case (ACCEL_ORIENT_TILTED_RIGHT): return "tilted right"; break;
        default: return "uknown";
    }
}

static char *accel_screen2name (int orient)
{
    switch (orient) {
        case (ACCEL_SCREEN_PORTRAIT): return "portrait"; break;
        case (ACCEL_SCREEN_LANDSCAPE): return "landscape"; break;
        default: return "uknown";
    }
}
/**
 * Irq bottom half handler
 *
 * @param work Bottom half structure
 */
void accel_irq_bottom_half_job(int force)
{
	int axes[3];
	int orientation=ACCEL_ORIENT_UNKNOWN, screen=0;
	int ax, ay;
    int ret;

    static unsigned int history_left = 0;
    static unsigned int history_right = 0;
    static unsigned int history_normal = 0;
    static unsigned int history_upside = 0;

    // debug
	//printk_orient ("status 0x%02x, src_1 0x%02x, src_2 0x%02x  force %d", 
    //    status, source1, source2,force);

    if (force) {
       /* clear input event to make input driver  deliver same event twice */
		input_report_abs (accel_info.idev, ABS_TILT_Y, 0);
		input_sync (accel_info.idev);
    }

	if (accel_events_mask & ACCEL_EV_SCREEN_ORIENT)
    {
    	if (accel_events_mask & ACCEL_EV_RAW_DATA)
        {
            // raw data mode is on - will get data from static buffer
            axes[0] = accel_info.x_axis;
            axes[1] = accel_info.y_axis;
            axes[2] = accel_info.z_axis;
        }
        else
        {
            // raw data mode is off - will directly read data from i2c
            ret = accel_i2c_read_xyz (axes);
            if (ret < 0)
            {
                // error
                //printk_data ("ERROR cannot read i2c...");
                return;
            }
        }

        history_left <<= 1;
        history_right <<= 1;
        history_normal <<= 1;
        history_upside <<= 1;

		ax=abs(axes[0]);
		ay=abs(axes[1]);
        if ((ax > 200) || (ay > 200))
        {
			if (ax > ay + 10) 
            {
				if (axes[0] < 0) 
                {
                    history_left |= 1;
                    //printk_data ("L: %x", history_left);
                    if((history_left & 0x0f) == 0x0f)
                    {
                        orientation = ACCEL_ORIENT_TILTED_LEFT;
                        history_left = 0;
                    }
                }
                else
                {
                    history_right |= 1;
                    //printk_data ("R: %x", history_right);
                    if((history_right & 0x0f) == 0x0f)
                    {
                        orientation = ACCEL_ORIENT_TILTED_RIGHT;
                        history_right = 0;
                    }
                }
			}
            else if (ay > ax + 10)
            {
				if (axes[1] < 0)
                {
                    history_normal |= 1;
                    //printk_data ("N: %x", history_normal);
                    if((history_normal & 0x0f) == 0x0f)
                    {
                        orientation = ACCEL_ORIENT_NORMAL;
                        history_normal = 0;
                    }
                }
                else
                {
                    history_upside |= 1;
                    //printk_data ("U: %x", history_upside);
                    if((history_upside & 0x0f) == 0x0f)
                    {
                        orientation = ACCEL_ORIENT_UPSIDE;
                        history_upside = 0;
                    }
                }
			}
        }
        else
        {
            //printk_data ("flat...");
        }

        // check orientation changed, then send event to framework
		if (orientation != ACCEL_ORIENT_UNKNOWN)
	    	    //&& (orientation != accel_info.device_orient))
		{
			switch (orientation) {
			 case ACCEL_ORIENT_NORMAL :
			 case ACCEL_ORIENT_UPSIDE :
				screen = ACCEL_SCREEN_PORTRAIT; 
					break;
			 case ACCEL_ORIENT_TILTED_LEFT :
			 case ACCEL_ORIENT_TILTED_RIGHT :
				screen = ACCEL_SCREEN_LANDSCAPE; 
					break;
			}

            // send event to framework
			input_report_abs (accel_info.idev, ABS_TILT_X, screen);
			input_report_abs (accel_info.idev, ABS_TILT_Y, orientation);
			input_sync (accel_info.idev);

			//mutex_lock (&accel_info.mlock);
			accel_info.screen_orient = screen; 
			accel_info.device_orient = orientation;
			//mutex_unlock (&accel_info.mlock);

            // debug
			printk_orient ("orientation 0x%04x (%s), screen 0x%02x (%s)", 
                orientation, accel_orient2name (orientation),
                screen, accel_screen2name (screen));
		}
	}
}
/**
 * Irq bottom half handler
 *
 * @param work Bottom half structure
 */
void accel_irq_bottom_half (struct work_struct *work)
{
    accel_irq_bottom_half_job(0);
    // schedule next work
    schedule_delayed_work(&accel_info.wq1, msecs_to_jiffies(ACCEL_ORIENT_POLL_INTERVAL));
}

static void accel_handle_data_ready (void)
{
	int	axes[3];
    int ret;

	ret = accel_i2c_read_xyz (axes);
    if (ret < 0) {
        return;
    }

	/* 
	 * Here we need to push data to the input if we want to use 
	 * data driven input rather than polled.
	 */
    input_report_abs (accel_info.idev, ABS_X, axes[0]);
    input_report_abs (accel_info.idev, ABS_Y, axes[1]);
    input_report_abs (accel_info.idev, ABS_Z, axes[2]);
    input_sync (accel_info.idev);
}

void accel_poll_data_ready (struct input_polled_dev *dev)
{
    int dr_enabled = atomic_read (&accel_info.data_ready_enabled);
    printk_poll ("data_ready_enabled is %d", dr_enabled);
    if (!dr_enabled) {
        return;
    }
    accel_handle_data_ready();
}

u8 accel_i2c_read_byte_data (u16 addr)
{
    s32 res;

    res = i2c_smbus_read_byte_data (accel_info.i2c, addr);
    if (res < 0) {
        fprintk ("error %d reading register 0x%X", res, addr);
        return 0;
    } else {
        return (u8)res;
    }
}

void accel_i2c_write_byte_data (u16 addr, u8 value)
{
    s32 res;

    res = i2c_smbus_write_byte_data (accel_info.i2c, addr, value);
    if (res < 0) {
        fprintk ("error %d writing 0x%X to register 0x%X", res, addr, value);
    }
}
