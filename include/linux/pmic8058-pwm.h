/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __PMIC8058_PWM_H__
#define __PMIC8058_PWM_H__

struct pw8058_pwm_config {
	int	pwm_size;	/* round up to 6 or 9 for 6/9-bit PWM SIZE */
	int	clk;
	int	pre_div;
	int	pre_div_exp;
	int	pwm_value;
	int	bypass_lut;
};

/* clk */
#define	PM_PWM_CLK_NO		0
#define	PM_PWM_CLK_1KHZ		1
#define	PM_PWM_CLK_32KHZ	2
#define	PM_PWM_CLK_19P2MHZ	3

/* pre-divide */
#define	PM_PWM_PREDIVIDE_2		0
#define	PM_PWM_PREDIVIDE_3		1
#define	PM_PWM_PREDIVIDE_5		2
#define	PM_PWM_PREDIVIDE_6		3

/* pre-divide exponent */
#define	PM_PWM_PREDIVIDE_EXP_MASK	0x07

int pwm_configure(struct pwm_device *pwm, struct pw8058_pwm_config *pwm_conf);

int pwm_set_dtest(struct pwm_device *pwm, int enable);

#if !defined(CONFIG_PMIC8058_PWM)
inline struct pwm_device *pwm_request(int pwm_id, const char *label)
{
	return NULL;
}

inline void pwm_free(struct pwm_device *pwm)
{
}

inline int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	return 0;
}

inline int pwm_enable(struct pwm_device *pwm)
{
	return 0;
}

inline void pwm_disable(struct pwm_device *pwm)
{
}

inline int pwm_configure(struct pwm_device *pwm,
			 struct pw8058_pwm_config *pwm_conf)
{
	return 0;
}

inline int pwm_set_dtest(struct pwm_device *pwm, int enable)
{
	return 0;
}
#endif

#endif /* __PMIC8058_PWM_H__ */
