/*
 * Copyright (C) 2008-2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

// Linux  driver for   ADP5588  keypad controller 

#include <linux/module.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/sysfs.h>
#include <linux/debugfs.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio_event.h>
#include <asm/gpio.h>
#include <linux/switch.h>

#include <asm/io.h>

#ifdef  CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/reboot.h>
#include <linux/syscalls.h>

#define  ADP5588NAME   "adp5588_keypad" 

extern unsigned long msleep_interruptible(unsigned int);

#define clk_busy_wait(time)	msleep_interruptible(time/1000)

#define    ADP5588_I2C_ADDRESS   0x34

#define    ADP5588_KEYPAD_FLIP_DETECT 42
#define    ADP5588_KEYPAD_RESET_GPIO  32
#define    ADP5588_MAX_KEYS           80


#define AD5588_GPIO_KEY             0x70        

#define    SW_MUTE		0x05  /* ringer mute  switch */


#define    ADP5588_COL_NUM    7
#define    ADP5588_ROW_NUM    8


/******************************************************************************
*  Registers IDs
******************************************************************************/
#define ADP5588_REG_DEV_ID                0x00  // Device ID
#define ADP5588_REG_CONFIG                0x01  // Config register
#define ADP5588_REG_INT_STAT              0x02  // Interrupt Status
#define ADP5588_REG_KEY_LCK_EC_STAT       0x03  // Key Lock and Event Counter Register
#define ADP5588_REG_KEY_EVENTA  	  0x04  // Key Event Register A
#define ADP5588_REG_KEY_EVENTB  	  0x05  // Key Event Register B
#define ADP5588_REG_KEY_EVENTC  	  0x06  // Key Event Register C
#define ADP5588_REG_KEY_EVENTD  	  0x07  // Key Event Register D
#define ADP5588_REG_KEY_EVENTE  	  0x08  // Key Event Register E
#define ADP5588_REG_KEY_EVENTF  	  0x09  // Key Event Register F
#define ADP5588_REG_KEY_EVENTG  	  0x0A  // Key Event Register G
#define ADP5588_REG_KEY_EVENTH  	  0x0B  // Key Event Register H
#define ADP5588_REG_KEY_EVENTI  	  0x0C  // Key Event Register I
#define ADP5588_REG_KEY_EVENTJ  	  0x0D  // Key Event Register J
#define ADP5588_REG_KP_LCK_TMR  	  0x0E  // Keypad Lock1 to Lock2 Timer
#define ADP5588_REG_UNLOCK1		  0x0F  // Unlock Key1
#define ADP5588_REG_UNLOCK2		  0x10  // Unlock Key2
#define ADP5588_REG_GPIO_INT_STAT1	  0x11  // GPIO Interrupt Status
#define ADP5588_REG_GPIO_INT_STAT2	  0x12  // GPIO Interrupt Status
#define ADP5588_REG_GPIO_INT_STAT3	  0x13  // GPIO Interrupt Status
#define ADP5588_REG_GPIO_DAT_STAT1	  0x14  // GPIO Data Status, Read twice to clear
#define ADP5588_REG_GPIO_DAT_STAT2	  0x15  // GPIO Data Status, Read twice to clear
#define ADP5588_REG_GPIO_DAT_STAT3	  0x16  // GPIO Data Status, Read twice to clear
#define ADP5588_REG_GPIO_DAT_OUT1	  0x17  // GPIO DATA OUT
#define ADP5588_REG_GPIO_DAT_OUT2	  0x18  // GPIO DATA OUT
#define ADP5588_REG_GPIO_DAT_OUT3	  0x19  // GPIO DATA OUT
#define ADP5588_REG_GPIO_INT_EN1	  0x1A  // GPIO Interrupt Enable
#define ADP5588_REG_GPIO_INT_EN2	  0x1B  // GPIO Interrupt Enable
#define ADP5588_REG_GPIO_INT_EN3	  0x1C  // GPIO Interrupt Enable
#define ADP5588_REG_KP_GPIO1		  0x1D  // Keypad or GPIO Selection
#define ADP5588_REG_KP_GPIO2		  0x1E  // Keypad or GPIO Selection
#define ADP5588_REG_KP_GPIO3		  0x1F  // Keypad or GPIO Selection
#define ADP5588_REG_GPI_EM1		  0x20  // GPI Event Mode 1
#define ADP5588_REG_GPI_EM2		  0x21  // GPI Event Mode 2
#define ADP5588_REG_GPI_EM3		  0x22  // GPI Event Mode 3
#define ADP5588_REG_GPIO_DIR1		  0x23  // GPIO Data Direction
#define ADP5588_REG_GPIO_DIR2		  0x24  // GPIO Data Direction
#define ADP5588_REG_GPIO_DIR3		  0x25  // GPIO Data Direction
#define ADP5588_REG_GPIO_INT_LVL1	  0x26  // GPIO Edge/Level Detect
#define ADP5588_REG_GPIO_INT_LVL2	  0x27  // GPIO Edge/Level Detect
#define ADP5588_REG_GPIO_INT_LVL3	  0x28  // GPIO Edge/Level Detect
#define ADP5588_REG_DEBOUNCE_DIS_1	  0x29  // Debounce Disable
#define ADP5588_REG_DEBOUNCE_DIS_2	  0x2A  // Debounce Disable
#define ADP5588_REG_DEBOUNCE_DIS_3	  0x2B  // Debounce Disable
#define ADP5588_REG_GPIO_PULL1  	  0x2C  // GPIO Pull Disable
#define ADP5588_REG_GPIO_PULL2  	  0x2D  // GPIO Pull Disable
#define ADP5588_REG_GPIO_PULL3  	  0x2E  // GPIO Pull Disable
#define ADP5588_REG_CMP_CFG_STAT	  0x30  // Comparator Configuration and Status Register
// registers 0x30 and up are not used. No light sensor.
		
		
/******************************************************************************
*  Chip Configuration 
******************************************************************************/
#define ADP5588_CONFIG_DEVICE	       0x31     //  NO I2C auto increment|Overflow ON| INT  re-assert|Key event INT
#define ADP5588_CONFIG_KEY_LOCK        0x00     //  Key lock Disable

#define ADP5588_CONFIG_GPIO_DAT_OUT3   0x00     //  Set EL_EN_1 and EL_EN_2 line to 0 by default

#define ADP5588_CONFIG_GPIO_INT2       0x80     //  Enable GPIO Interrupt for RINGER/SILENT KEY


#define ADP5588_CONFIG_KP_GPIO1        0xff     //  Row0 to Row7  are KP Matrix
#define ADP5588_CONFIG_KP_GPIO1_M      0xfe     //  Row0 to Row7  are KP Matrix
#define ADP5588_CONFIG_KP_GPIO1_M_LOCK 0x28     //  Row3 and Row5  only  are KP Matrix

#define ADP5588_CONFIG_KP_GPIO2        0x7f     //  Col0 to Col6  are KP Matrix | Col7  is GPIO 
#define ADP5588_CONFIG_KP_GPIO2_M_LOCK 0x05     //  Col0 and Col2  only are KP Matrix 


#define ADP5588_CONFIG_KP_GPIO3       0x00     //  COl8 & Col9  are GPIOs
#define ADP5588_CONFIG_GPI_EM2        0x80     //  GPI  on Col7 ( RINGER/SILENT KEY ) event is part of event FIFO


#define ADP5588_CONFIG_GPIO_DIR3      0x03     //  COl8 & Col9  GPIOs  are outputs  ( EL_EN_1 and EL_EN_2 )
#define ADP5588_CONFIG_INT_LVL2       0x00     //  GPI  on Col7 ( RINGER/SILENT KEY ) INT is LOW 
#define ADP5588_CONFIG_GPIO_PULL2     0x00     //  GPI  on Col7 ( RINGER/SILENT KEY ) pull UP
 
 
/******************************************************************************
*  Key Data FLAGS  
******************************************************************************/
#define ADP5588_KEY_RELEASE         0x80     //  Key release BIT
#define ADP5588_MAX_KEY_CODE        0x50     //  MAX Key code 
#define ADP5588_KEY_CODE            0x7f     //  Key code mask 
#define ADP5588_MAX_GPIO_EVENT      0x72     //  MAX GPIO EVENT  code 
#define ADP5588_KEY_FIFO_EMPTY      0x00     //  Queue is empty
#define ADP5588_RINGER_KEY_BIT      0x80     //  A bit in ADP5588_REG_GPIO_DAT_STAT2 reg for col7 (RINGER/SILENT KEY) 

 
   typedef struct
   {
    uint16_t	      reg;
    uint8_t	      data;
   } ADP5588_cfg;



static ADP5588_cfg adp5588_init_seq[] =
{
    { ADP5588_REG_CONFIG,             ADP5588_CONFIG_DEVICE,  },    /* NO I2C auto increment|Overflow ON| INT  re-assert|Key event INT */
    { ADP5588_REG_KEY_LCK_EC_STAT,    ADP5588_CONFIG_KEY_LOCK,},     /* Key lock Disable*/
    { ADP5588_REG_GPIO_DAT_OUT3,      ADP5588_CONFIG_GPIO_DAT_OUT3,},/*  Set EL_EN_1 and EL_EN_2 line to 0 by default*/
    { ADP5588_REG_GPIO_INT_EN2,       ADP5588_CONFIG_GPIO_INT2,},   /*  Enable GPIO Interrupt for RINGER/SILENT KEY*/
       /*  Configure  8x7 Keypad  Matrix*/
    { ADP5588_REG_KP_GPIO1,	      ADP5588_CONFIG_KP_GPIO1,},    /*  Row0 to Row7  are KP Matrix*/
    { ADP5588_REG_KP_GPIO2,	      ADP5588_CONFIG_KP_GPIO2,},    /*  Col0 to Col6  are KP Matrix | Col7  is GPIO*/
    { ADP5588_REG_KP_GPIO3,	      ADP5588_CONFIG_KP_GPIO3,},    /*  COl8 & Col9  are GPIOs */
    { ADP5588_REG_GPI_EM2,	      ADP5588_CONFIG_GPI_EM2   },   /* GPI  on Col7 ( RINGER/SILENT KEY ) event is part of event FIFO */
    { ADP5588_REG_GPIO_DIR3,	      ADP5588_CONFIG_GPIO_DIR3 },   /* COl8 & Col9  GPIOs  are outputs  ( EL_EN_1 and EL_EN_2 ) */
    { ADP5588_REG_GPIO_INT_LVL2,      ADP5588_CONFIG_INT_LVL2  },   /*  GPI  on Col7 ( RINGER/SILENT KEY ) INT is LOW  */
    { ADP5588_REG_GPIO_PULL2,         ADP5588_CONFIG_GPIO_PULL2},   /*  GPI  on Col7 ( RINGER/SILENT KEY ) pull UP */  
    {ADP5588_REG_INT_STAT,            0x5},                           /* Clear  Interrupt */
    /* Must end with 0 reg */
    { 0,          0x00 },
};	 

static ADP5588_cfg adp5588_init_seq_motus[] =
{
    { ADP5588_REG_CONFIG,             ADP5588_CONFIG_DEVICE,  },    /* NO I2C auto increment|Overflow ON| INT  re-assert|Key event INT */
    { ADP5588_REG_KEY_LCK_EC_STAT,    ADP5588_CONFIG_KEY_LOCK,},     /* Key lock Disable*/
    { ADP5588_REG_GPIO_DAT_OUT3,      ADP5588_CONFIG_GPIO_DAT_OUT3,},/*  Set EL_EN_1 and EL_EN_2 line to 0 by default*/
       /*  Configure  8x7 Keypad  Matrix*/
    { ADP5588_REG_KP_GPIO1,	      ADP5588_CONFIG_KP_GPIO1_M,},    /*  Row1 to Row7  are KP Matrix*/
    { ADP5588_REG_KP_GPIO2,	      ADP5588_CONFIG_KP_GPIO2,},    /*  Col0 to Col6  are KP Matrix | Col7  is GPIO*/
    { ADP5588_REG_KP_GPIO3,	      ADP5588_CONFIG_KP_GPIO3,},    /*  COl8 & Col9  are GPIOs */
    {ADP5588_REG_INT_STAT,            0x5 },                           /* Clear  Interrupt */
    /* Must end with 0 reg */
    { 0,          0x00 },
};	 
static ADP5588_cfg adp5588_init_seq_motus_lock[] =
{
    { ADP5588_REG_CONFIG,             ADP5588_CONFIG_DEVICE,  },    /* NO I2C auto increment|Overflow ON| INT  re-assert|Key event INT */
    { ADP5588_REG_KEY_LCK_EC_STAT,    ADP5588_CONFIG_KEY_LOCK,},     /* Key lock Disable*/
    { ADP5588_REG_GPIO_DAT_OUT3,      ADP5588_CONFIG_GPIO_DAT_OUT3,},/*  Set EL_EN_1 and EL_EN_2 line to 0 by default*/
       /*  Configure  8x7 Keypad  Matrix*/
    { ADP5588_REG_KP_GPIO1,	      ADP5588_CONFIG_KP_GPIO1_M_LOCK,},    /*  Row3 to Row5  only*/
    { ADP5588_REG_KP_GPIO2,	      ADP5588_CONFIG_KP_GPIO2_M_LOCK,},    /*  Col0 to Col2  only*/
    { ADP5588_REG_KP_GPIO3,	      ADP5588_CONFIG_KP_GPIO3,},    /*  COl8 & Col9  are GPIOs */
    {ADP5588_REG_INT_STAT,            0x5 },                           /* Clear  Interrupt */
    /* Must end with 0 reg */
    { 0,          0x00 },
};	 

static ADP5588_cfg adp5588_lock_seq_motus[] =
{
       /*  Configure  2x2 Keypad  Matrix*/
    { ADP5588_REG_KP_GPIO1,	      ADP5588_CONFIG_KP_GPIO1_M_LOCK,},    /*  Row3 to Row5  only*/
    { ADP5588_REG_KP_GPIO2,	      ADP5588_CONFIG_KP_GPIO2_M_LOCK,},    /*  Col0 to Col2  only*/
     {ADP5588_REG_INT_STAT,            0x5 },                           /* Clear  Interrupt */
    /* Must end with 0 reg */
    { 0,          0x00 },
};	 
static ADP5588_cfg adp5588_unlock_seq_motus[] =
{
       /*  Configure  8x7 Keypad  Matrix*/
    { ADP5588_REG_KP_GPIO1,	      ADP5588_CONFIG_KP_GPIO1_M,},    /*  Row1 to Row7  are KP Matrix*/
    { ADP5588_REG_KP_GPIO2,	      ADP5588_CONFIG_KP_GPIO2,},    /*  Col0 to Col6  are KP Matrix | Col7  is GPIO*/
    {ADP5588_REG_INT_STAT,            0x5 },                           /* Clear  Interrupt */
    /* Must end with 0 reg */
    { 0,          0x00 },
};	 

   typedef  struct
   {
   uint8_t          reg;
   uint8_t          data;
   } ADP5588_CMD_T;




static struct adp5588_keypad_data {
	uint16_t            addr;
	struct i2c_client   *client;
	struct input_dev   *input_dev;
	struct input_dev   *sw_dev;
	uint8_t             use_irq;
	uint8_t             dev_id;
	struct              hrtimer timer;
	struct work_struct  work;
	struct work_struct  lock_work;
	uint16_t            last_key;
	uint16_t            last_key_state;
	uint8_t             ringer_switch;
	uint8_t             device_type;
	int                 update_config;
    struct mutex	    mutex;		
	struct switch_dev    sdev;	
#ifdef  CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
}  adp5588_data;

#define   I2C_MUTEX_LOCK        mutex_lock(&adp5588_data.mutex)
#define   I2C_MUTEX_UNLOCK        mutex_unlock(&adp5588_data.mutex)
//#define   I2C_MUTEX_LOCK         printk("ADP5588   mutex_lock")
//#define   I2C_MUTEX_UNLOCK       printk("ADP5588   mutex_UNlock")

#ifdef  CONFIG_HAS_EARLYSUSPEND
static void adp5588_early_suspend(struct early_suspend *handler);
static void adp5588_early_resume(struct early_suspend *handler);
#endif

void  set_lid_state( int lid , int tt )
{
switch_set_state(&adp5588_data.sdev, tt<<1|lid);
}

static void switch_event(struct input_handle *handle, unsigned int type, unsigned int code, int value)
{

	if (type == EV_SW && code == SW_TABLET_MODE )
	{
	  printk(KERN_DEBUG "%s(%s): Event.  Type: %d, Code: %d, Value: %d   KEYPAD  : %s \n",__FILE__, __FUNCTION__,
		 type, code, value , value ? "UNLOCKED":"LOCKED");

        adp5588_data.update_config = value+1;
	    schedule_work(&adp5588_data.lock_work);
	}


	if (type == EV_SW && ((code == SW_TABLET_MODE) || (code == SW_LID)))
            switch_set_state(&adp5588_data.sdev,(switch_get_state(&adp5588_data.sdev) & ~(1<<code)) | ( value << code) );
}

static int switch_connect(struct input_handler *handler, struct input_dev *dev,
			 const struct input_device_id *id)
{
	struct input_handle *handle;
	int error ,sw_value;

	if(!test_bit(SW_TABLET_MODE , dev->swbit) )   return -ENODEV;

	if( adp5588_data.input_dev == dev )   return -ENODEV;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "keypad_lock_switch";
	adp5588_data.sw_dev = dev;
	error = input_register_handle(handle);
	if (error)
		goto err_free_handle;

	error = input_open_device(handle);
	if (error)
		goto err_unregister_handle;
        sw_value = !!test_bit(SW_TABLET_MODE, dev->sw);

        set_lid_state( !!test_bit(SW_LID, dev->sw),sw_value);
	printk(KERN_DEBUG "%s(%s): Connected device: \"%s\", %s\n  sw state  %d ",__FILE__, __FUNCTION__, dev->name, dev->phys,sw_value);

    adp5588_data.update_config = sw_value+1;
	schedule_work(&adp5588_data.lock_work);


	return 0;

 err_unregister_handle:
	input_unregister_handle(handle);
 err_free_handle:
	kfree(handle);
	return error;
}

static void switch_disconnect(struct input_handle *handle)
{
	printk(KERN_DEBUG "%s(%s): Disconnected device: %s\n", __FILE__, __FUNCTION__,handle->dev->phys);

	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id switch_ids[] = {
	{
                .flags = INPUT_DEVICE_ID_MATCH_EVBIT,
                .evbit = { BIT_MASK(EV_SW) },
        },
	{ },			/* Terminating zero entry */
};

MODULE_DEVICE_TABLE(input, switch_ids);

static struct input_handler switch_handler = {
	.event =	switch_event,
	.connect =	switch_connect,
	.disconnect =	switch_disconnect,
	.name =		"keypad_lock_switch",
	.id_table =	switch_ids,
};


static int  adp5588_read_register(struct i2c_client *client,uint8_t address , uint8_t *value )
{
  uint8_t data[1];
  int	rc;
 
  I2C_MUTEX_LOCK;

  data[0] = address;  //reg 0 
  if ((rc = i2c_master_send(client, data, 1)) < 0)
     {
     dev_err(&client->dev," %s(%s):i2c_master_send error %d\n",
		            __FILE__, __FUNCTION__, rc);
     I2C_MUTEX_UNLOCK;
     return rc;
     }
   
   clk_busy_wait(1000);

   *value=0;
   if ((rc = i2c_master_recv(client, value,1 )) < 0 )
      dev_err(&client->dev," %s(%s):i2c_master_recv error %d\n",
		            __FILE__, __FUNCTION__, rc);
     I2C_MUTEX_UNLOCK;
return(rc);     
}

static int adp5588_write_register(struct i2c_client *client,uint8_t address , uint8_t value )
{
	int rc;
	ADP5588_CMD_T  msg;

 	msg.reg = address;
        msg.data = value; 
        I2C_MUTEX_LOCK;
	if ((rc = i2c_master_send(client, (uint8_t *)&msg,sizeof(msg))) < 0)
	   dev_err(&client->dev," %s(%s):i2c_master_send error %d\n",
		            __FILE__, __FUNCTION__, rc);
        I2C_MUTEX_UNLOCK;	
        return rc;
}

/* adp5588 write configuration */
static int adp5588_write_config(struct i2c_client *client,ADP5588_cfg   *cfg)
{
	int ret=0 ,i;
	ADP5588_CMD_T  msg;

        if( !client || !cfg ) return -1;
        I2C_MUTEX_LOCK;
        for( i =0;cfg[i].reg !=0; i++ ) 
	{
	
	msg.reg = cfg[i].reg;
        msg.data = cfg[i].data;
	if ((ret = i2c_master_send(client, (uint8_t *)&msg,sizeof(msg))) < 0)
		 dev_err(&client->dev," %s(%s):i2c_master_send error %d\n",
		         __FILE__, __FUNCTION__, ret);
	 clk_busy_wait(1000);
        }
        I2C_MUTEX_UNLOCK;		
	return ret;
}


/* adp5588 initial configuration */

static int adp5588_config(struct i2c_client *client)
{
   clk_busy_wait(1000);
	return(adp5588_write_config ( client,  adp5588_data.device_type  ? adp5588_init_seq_motus:adp5588_init_seq));	  
}

/* adp5588 lock/unlock  motus keypad */

static int adp5588_lock_motus_keypad( int  state  )
{
        return(adp5588_write_config ( adp5588_data.client,  state  ? adp5588_unlock_seq_motus:adp5588_lock_seq_motus));	 
}


static int adp5588_clear_irq(struct i2c_client *client)
{
	 // clear  key event & key lock irq
	return( adp5588_write_register(client,ADP5588_REG_INT_STAT,0x5 ));
}

/* SET  backlight drive register . Bit 0  = EL_EN1  , Bit 1 =EL_EN2  */

int adp5588_set_backlight( uint8_t mask  )
{
	if ( adp5588_data.client)   
	    return( adp5588_write_register(adp5588_data.client,ADP5588_REG_GPIO_DAT_OUT3,mask ));
	return -ENOTSUPP;
}

EXPORT_SYMBOL(adp5588_set_backlight);


/* GET   backlight drive register  value. Bit 0  = EL_EN1  , Bit 1 =EL_EN2  */

int adp5588_get_backlight()
{
	uint8_t mask;
	int rc;
	if ( adp5588_data.client)   
	  {	
        	if ( (rc=adp5588_read_register(adp5588_data.client,
		       ADP5588_REG_GPIO_DAT_OUT3 ,&mask ))< 0 ) 
		      return  rc;
	 	else  return  mask ;
          }
	return -ENOTSUPP;
}

EXPORT_SYMBOL(adp5588_get_backlight);

static void adp5588_lock_work_func( struct work_struct *work)
{
	struct adp5588_keypad_data *kp = container_of(work, struct adp5588_keypad_data, lock_work);
      /*  Keypad lock switch changed. update ADP5588 configuration */
      if( adp5588_data.update_config ) 
	   { 
            adp5588_lock_motus_keypad(  adp5588_data.update_config -1  );
            adp5588_data.update_config =0;
       }
}


static void adp5588_work_func( struct work_struct *work)
{
	int rc,i;
	uint8_t reg=ADP5588_REG_KEY_EVENTA;
	struct adp5588_keypad_data *kp = container_of(work, struct adp5588_keypad_data, work);
	struct i2c_client *client=kp->client;
        uint8_t  scan_code;		


        /* set  read address */

         I2C_MUTEX_LOCK;

         if ((rc = i2c_master_send(client, &reg, 1)) <0)     
	   {
	    dev_err(&client->dev," %s(%s):i2c_master_send error %d\n",
		         __FILE__, __FUNCTION__, rc);
           /* I2c Write error  .  exit now , enable IRQ and read again if IRQ pin  still low */  

	  }else {
        
        /* read scancodes until queue is empty */
        i=0;
        do {
        scan_code =ADP5588_KEY_FIFO_EMPTY;

        if ((rc = i2c_master_recv(client, &scan_code,1 )) < 0 )
	   {
	    dev_err(&client->dev," %s(%s):i2c_master_recv error %d\n",
		         __FILE__, __FUNCTION__, rc);
	    break;
	   }
	        
	  if( scan_code != ADP5588_KEY_FIFO_EMPTY )
	  {	  
	  kp->last_key = ADP5588_KEY_CODE & scan_code ;	
	  kp->last_key_state  = !!(ADP5588_KEY_RELEASE & scan_code); 

	  dev_dbg(&client->dev,"ADP5588 got scancode %d ,keycode 0x%x ( %d ) state %d \n",

	  scan_code, kp->last_key, kp->last_key,kp->last_key_state ); 
	  if(  kp->last_key == AD5588_GPIO_KEY )
	  {
	  /* got   RINGER/SILENCE  switch event */
          adp5588_data.ringer_switch = kp->last_key_state;	   
	  input_report_switch(kp->input_dev, SW_MUTE,adp5588_data.ringer_switch );
	  } else {
	  /* got   regular key event */	  
	  input_report_key(kp->input_dev, kp->last_key, kp->last_key_state);
	  }
//  Sync is removed.  It does not required  for key event				 
//	  input_sync(kp->input_dev);	   
	 }else  if( i == 0 ) { /*IRQ without the code ? , must be  key release  HW problem , Wait  5ms  */ 
         
            clk_busy_wait(5000);
          }
          
	} while( scan_code && (i++ <10) );
       }
        I2C_MUTEX_UNLOCK;

       /* Clear IRQ and enable it */
	if (kp->use_irq){
          if ( adp5588_clear_irq(client)<0) 
            { /* I2c Problem , do not enable IRQ but wait 1 sec */
	    hrtimer_start(&kp->timer, ktime_set(1,0), HRTIMER_MODE_REL);
            }else  enable_irq(client->irq);
       }

}

/* adp5588 retry work function */

static enum hrtimer_restart adp5588_retry_work(struct hrtimer *timer)
{
	struct adp5588_keypad_data *kp = container_of(timer,struct adp5588_keypad_data, timer);	

	schedule_work(&kp->work);

	return HRTIMER_NORESTART;
}

/* adp5588 timer handler */

static enum hrtimer_restart adp5588_timer_func(struct hrtimer *timer)
{
	struct adp5588_keypad_data *kp = container_of(timer,struct adp5588_keypad_data, timer);	

	schedule_work(&kp->work);

	hrtimer_start(&kp->timer, ktime_set(0, 125000000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}


/* adp5588 interrupt handler */

static irqreturn_t adp5588_irq_handler(int irq, void *dev_id)
{
	struct adp5588_keypad_data *kp = (struct adp5588_keypad_data *)dev_id;
	disable_irq(irq);
	schedule_work(&kp->work);
	return IRQ_HANDLED;
}


static ssize_t print_lid_switch_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
	case 1:   // CLOSED
		return sprintf(buf, "Closed\n");
	case 2:    // Open  
		return sprintf(buf, "Open\n");
	case 0:    // Table top   
		return sprintf(buf, "Table top\n");
	}
	return -EINVAL;
}

/* This function is called by i2c_probe */
static int adp5588_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0,i,j;
	uint8_t  reg;
	unsigned long request_flags =  IRQF_TRIGGER_LOW;
	struct platform_device *pdev;
		
	/* GET  ADP5588 OUT of RESET   */ 
/*  done in board-mot.c 	
	 err = gpio_request(ADP5588_KEYPAD_RESET_GPIO, "adp5588_nRST");
	 if (err) {
		 dev_err(&client->dev," %s(%s):Can't request gpio %d\n",
		         __FILE__, __FUNCTION__, ADP5588_KEYPAD_RESET_GPIO);
		 return err;
	 }
	 err=gpio_direction_output(ADP5588_KEYPAD_RESET_GPIO, 1);	// enable chip nRST
	 if (err) {
                 dev_err(&client->dev," %s(%s):Can't config output gpio %d\n",
		         __FILE__, __FUNCTION__, ADP5588_KEYPAD_RESET_GPIO);
		 goto exit_gpio_free;
	 }
*/
	mutex_init( &adp5588_data.mutex );


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		 dev_err(&client->dev,
			"No supported i2c func what we need?!!\n");
		 err =  -ENOTSUPP;
		 goto exit_gpio_free;		
	}


        pdev = (struct platform_device *)client->dev.platform_data;

	adp5588_data.client = client ;
	i2c_set_clientdata(client, &adp5588_data);

	/*  device type : 0 - morrison , 1 - motus */
        adp5588_data.device_type = pdev->id;
        /* Configure   APD5588 chip */


        adp5588_config( client );
        
	/* Get  Chip ID */ 
        adp5588_read_register(client, ADP5588_REG_DEV_ID ,&adp5588_data.dev_id );
        
	printk("ADP5588 Keypad Dev ID : 0x%x,Body ID %d  Name : %s , RST %d\n",adp5588_data.dev_id,adp5588_data.device_type,pdev->name,gpio_get_value(ADP5588_KEYPAD_RESET_GPIO));
	
        INIT_WORK(&adp5588_data.work, adp5588_work_func);

        INIT_WORK(&adp5588_data.lock_work, adp5588_lock_work_func);

        /* Configure and  Register  Linux Input Device  */
	adp5588_data.input_dev = input_allocate_device();
	if (adp5588_data.input_dev == NULL) {
		err = -ENOMEM;
                dev_err(&client->dev," %s(%s):Can't allocate input device \n",
		         __FILE__, __FUNCTION__);
		goto err_input_dev_alloc_failed;
	}
//	adp5588_data.input_dev->name = ADP5588NAME;
        adp5588_data.input_dev->name = pdev->name;
	adp5588_data.input_dev->id.bustype = BUS_I2C;
	adp5588_data.input_dev->id.vendor = adp5588_data.dev_id>>4;
	adp5588_data.input_dev->id.product = adp5588_data.dev_id|0xf;
	adp5588_data.input_dev->id.version = 0x0100;

	
	set_bit(EV_KEY, adp5588_data.input_dev->evbit);
	set_bit(EV_SW,  adp5588_data.input_dev->evbit);



      for(i = 0; i < ADP5588_ROW_NUM; i++) 
       for( j =1 ; j <= ADP5588_COL_NUM ; j++) {
	   set_bit(j+ i*10 , adp5588_data.input_dev->keybit);   
      }
 
#if defined(CONFIG_DEBUG_FS)
/* test only :  Add GPIO keys to be able report them for testing */

	set_bit(KEY_MENU , adp5588_data.input_dev->keybit);  
	set_bit(KEY_POWER , adp5588_data.input_dev->keybit);   
	set_bit(KEY_HOME , adp5588_data.input_dev->keybit);  
	set_bit(KEY_BACK , adp5588_data.input_dev->keybit);  
	set_bit(KEY_MEDIA , adp5588_data.input_dev->keybit);  
	set_bit(SW_LID, adp5588_data.input_dev->swbit); 	
	if ( adp5588_data.device_type )
	{  /* MOTUS */
	set_bit(SW_TABLET_MODE , adp5588_data.input_dev->swbit);
	 }
#endif
 
   
  
	set_bit(SW_MUTE, adp5588_data.input_dev->swbit);
	
	err = input_register_device(adp5588_data.input_dev);
	
	if (err) {
		dev_err(&client->dev," %s(%s): Unable to register %s input device\n",
		 __FILE__, __FUNCTION__,adp5588_data.input_dev->name);
		goto err_input_register_device_failed;
	}


	
	if (client->irq) {
	  err = request_irq(client->irq, adp5588_irq_handler, request_flags, ADP5588NAME,&adp5588_data);
	  if (err == 0)
	  {
	   adp5588_data.use_irq = 1;
           err = set_irq_wake(client->irq, 1);
	  }else {
	  dev_err(&client->dev, " %s(%s):request irq  %d  failed\n",
	           __FILE__, __FUNCTION__,client->irq);
		adp5588_data.use_irq = 0;
		free_irq(client->irq, &adp5588_data);		
		}
	}


		hrtimer_init(&adp5588_data.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	if (!adp5588_data.use_irq) {
		adp5588_data.timer.function = adp5588_timer_func;
		hrtimer_start(&adp5588_data.timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}else  {
		adp5588_data.timer.function = adp5588_retry_work;
        }
	printk(KERN_INFO "ADP5588 : Start keypad %s in %s mode irq %d \n",
	                           adp5588_data.input_dev->name,
				   adp5588_data.use_irq ? "interrupt" : "polling",client->irq);
	if ( !adp5588_data.device_type )	
         {

         if ( adp5588_read_register(client, ADP5588_REG_GPIO_DAT_STAT2 ,&reg )< 0 )
	 {  
	  adp5588_data.ringer_switch =1; 
	  dev_err(&client->dev," %s(%s):Can't read Ringer/Silent switch state.Set it to ON\n",
	           __FILE__, __FUNCTION__ );
	 }else {      
         adp5588_data.ringer_switch = !(ADP5588_RINGER_KEY_BIT & reg ) ;
         
	 }
	 dev_dbg(&client->dev," %s(%s):Ringer/Silent switch is %s\n",
	           __FILE__, __FUNCTION__, adp5588_data.ringer_switch ? "ON":"MUTE" );
     	  }else  /* No MUTE switch on Motus , set to default */             
	    adp5588_data.ringer_switch =1; 
	  
	 input_report_switch(adp5588_data.input_dev, SW_MUTE,adp5588_data.ringer_switch );
	 input_sync(adp5588_data.input_dev); 
#if defined(CONFIG_DEBUG_FS)
    	 input_report_switch(adp5588_data.input_dev, SW_LID,!gpio_get_value(ADP5588_KEYPAD_FLIP_DETECT ));
	 input_sync(adp5588_data.input_dev); 
#endif


	if ( adp5588_data.device_type )  /* it is MOTUS  .  start lock switch handler */	
         {
	adp5588_data.sdev.name = "lid";
	adp5588_data.sdev.print_name = print_lid_switch_name;


	err = switch_dev_register(&adp5588_data.sdev);
	if (err < 0)
		goto err_switch_dev_register;


       	 err =  input_register_handler(&switch_handler);  
	 if (err) {
		dev_err(&client->dev," %s(%s): Unable to register  key lock  input handler e\n",
		 __FILE__, __FUNCTION__);
		goto err_input_register_device_failed;
	  }
         }


 
#ifdef  CONFIG_HAS_EARLYSUSPEND
    // Level should be higher than keypad backlight driver!
	adp5588_data.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	adp5588_data.early_suspend.suspend = adp5588_early_suspend;
	adp5588_data.early_suspend.resume = adp5588_early_resume;
	register_early_suspend(&(adp5588_data.early_suspend));
#endif


	return 0;
err_switch_dev_register :
err_input_register_device_failed:
	if ( adp5588_data.device_type ) switch_dev_unregister(&adp5588_data.sdev);
	input_free_device(adp5588_data.input_dev);
err_input_dev_alloc_failed:
exit_gpio_free:
	 gpio_free( ADP5588_KEYPAD_RESET_GPIO);
	return err;
}


#if defined(CONFIG_DEBUG_FS)
#define TEST_SCANCODE_GET(code)         (code&0xffff)
#define TEST_IS_SWITCH(code)            (code&0xff00)
#define TEST_SWITCH_GET(code)           (code&0xff)
#define TEST_STATE_GET(code)           ((code>>16)& 1)
#define TEST_SCANCODE_SET(code,state,lid,ringer)  ((state<<16)| code|(lid<<8)|(ringer<13))
#define    MORRISON_SLIDER_DETECT_GPIO 42

 



static int adp5588_debug_set(void *data, u64 val)
{
         dev_dbg(&(adp5588_data.client->dev)," %s(%s):code %llu , state %llu \n",
            __FILE__, __FUNCTION__, TEST_SCANCODE_GET(val),TEST_STATE_GET(val));
         if( TEST_IS_SWITCH(val) ) 
	 {
	 	/* Simulate a switch  event */
	 input_report_switch(adp5588_data.input_dev,TEST_SWITCH_GET(val),TEST_STATE_GET(val));	
		
         }else{
	/* Simulate a  key press/release  */	
	input_report_key(adp5588_data.input_dev,TEST_SCANCODE_GET(val),TEST_STATE_GET(val));		       
//  Sync is removed.  It does not required  for key event	
//	input_sync(adp5588_data.input_dev);	
         } 
	return 0;
}

static int adp5588_debug_get(void *data,u64 *val)
{
        /* return last keypress */
	*val= TEST_SCANCODE_SET( adp5588_data.last_key, adp5588_data.last_key_state,
	                         gpio_get_value( MORRISON_SLIDER_DETECT_GPIO ),adp5588_data.ringer_switch );		       
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(adp5588_debug_fops, adp5588_debug_get, adp5588_debug_set, "%llu\n");

static int __init adp5588_debug_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("adp5588", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("scancode", 0644, dent, NULL, &adp5588_debug_fops);

	return 0;
}

device_initcall(adp5588_debug_init);
#endif




static int adp5588_remove(struct i2c_client *client)
{
	struct adp5588_keypad_data *kp = i2c_get_clientdata(client);
	dev_dbg(&client->dev, "%s: enter.\n", __FUNCTION__);
	if ( adp5588_data.device_type )  /* it is MOTUS  */	
 	{ switch_dev_unregister(&adp5588_data.sdev);
          input_unregister_handler(&switch_handler);
        }

//	if (kp->use_irq)
//	{
		free_irq(client->irq, kp);
//        }else
		hrtimer_cancel(&kp->timer);
        gpio_free( ADP5588_KEYPAD_RESET_GPIO);
	input_unregister_device(kp->input_dev);
	kfree(kp);
	return 0;
}

static int adp5588_suspend(struct i2c_client *client, pm_message_t mesg)
{
	client=client;
	mesg=mesg;
 //      	printk( "%s: enter. \n", __FUNCTION__);
	return (0);
}

static int adp5588_resume(struct i2c_client *client)
{
	int lock=0 ;
	struct adp5588_keypad_data *kp = i2c_get_clientdata(client);
    if( kp->device_type && adp5588_data.sw_dev)
         lock = !test_bit(SW_TABLET_MODE, adp5588_data.sw_dev->sw);
//  printk( "%s: enter.  lock %d , type %d   sw_dev %x\n", __FUNCTION__,lock,kp->device_type,adp5588_data.sw_dev);
	adp5588_write_config ( client,  kp->device_type  ? ( lock ? adp5588_init_seq_motus_lock: adp5588_init_seq_motus):adp5588_init_seq);	  

return(0);
}


#ifdef  CONFIG_HAS_EARLYSUSPEND
static void adp5588_early_suspend(struct early_suspend *handler)
{
//	dev_dbg(NULL, "%s: enter.\n", __FUNCTION__);
        adp5588_suspend(adp5588_data.client, PMSG_SUSPEND);
	
}

static void adp5588_early_resume(struct early_suspend *handler)
{
//	dev_dbg(NULL, "%s: enter.\n", __FUNCTION__);
        adp5588_resume(adp5588_data.client);

}
#endif
static const struct i2c_device_id adp5588_id[] = {
	{ ADP5588NAME, 0 },
	{ }
};

/* This is the driver that will be inserted */
static struct i2c_driver adp5588_keypad_driver = {
#ifndef  CONFIG_HAS_EARLYSUSPEND
	.suspend	= adp5588_suspend,
	.resume		= adp5588_resume,
#endif
	.probe		= adp5588_probe,
	.remove		= adp5588_remove,
	.id_table = adp5588_id,
	.driver		= {
		.name = ADP5588NAME,
	},
};


static int __devinit adp5588_init(void)
{
return 	i2c_add_driver(&adp5588_keypad_driver);
}


static void __exit adp5588_exit(void)
{
	i2c_del_driver(&adp5588_keypad_driver);
}


device_initcall(adp5588_init);
module_exit(adp5588_exit);

MODULE_DESCRIPTION("ADP5588 KEYPAD DRIVER");
MODULE_AUTHOR("Vladimir Karpovich <Vladimir.Karpovich@motorola.com>");
MODULE_LICENSE("GPL");





