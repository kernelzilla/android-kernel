#ifndef __LEDS_LD_CPCAP_DISP
#define __LEDS_LD_CPCAP_DISP

#include <linux/spi/cpcap.h>

struct disp_button_config_data {
	u16 duty_cycle;
	u16 cpcap_mask;
	u16 abmode_cpcap_mask;
	u16 pwm;
	u16 fade_time;
	u16 fade_en;
	u16 abmode;
	u8 led_current;
	enum cpcap_reg reg;
	enum cpcap_reg reg2;
};

#endif
