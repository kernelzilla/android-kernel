/*
 * include/lp3907_i2c.h
 *
 * Configuration for LP3907 TDMB tuner regulator
 */

#ifndef __LINUX_LP3907_I2C_H
#define __LINUX_LP3907_I2C_H

#include <linux/ioctl.h>

/*
typedef struct {

} __attribute__((packed)) lp3907_info;
*/

struct lp3907_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};

/* LP3907 indicator */
#define LP3907_DRIVER_NAME 	     "lp3907"
#define LP3907_DEVICE_NAME         "/dev/lp3907"

/* ****************************************************************************
 *   Command ID
 * ************************************************************************** */

#define LP3907_MAGIC    'l'

#define LP3907_PWR_OFF_CMD          _IO(LP3907_MAGIC, 0)
#define	LP3907_PWR_ON_CMD    	      _IO(LP3907_MAGIC, 1)

#define LP3907_MAXNR                 2


#endif /* __LINUX_LP3907_I2C_H */
