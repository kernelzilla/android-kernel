/****************************************************************

Siano Mobile Silicon, Inc.
MDTV receiver kernel modules.
Copyright (C) 2006-2008, Uri Shkolnik

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

 This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

****************************************************************/

#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/irq.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/dma-mapping.h>
#include <asm/dma.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/vmalloc.h>
#include <linux/i2c.h>

#include "smsdbg_prn.h"
#include "smscoreapi.h"

#include <linux/time.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/clock.h>

#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <mach/system.h>
#include <mach/dma.h>

#include <linux/gpio_mapping.h>
#include <mach/mux.h>

/* debug macro */
#define tdmblog(fmt, arg...) printk(KERN_DEBUG "TDMB_SEQ: "fmt, ##arg)

#define SPI_PACKET_SIZE 256

/*
#define MDTV_INT_N_GPIO         38
#define MDTV_PWDN_GPIO          53
#define MDTV_RESET_N_GPIO       54
*/

#define MDTV_INT_N_GPIO         get_gpio_by_name("mtv_int")
#define MDTV_PWDN_GPIO          get_gpio_by_name("mtv_pwdn")
#define MDTV_RESET_N_GPIO       get_gpio_by_name("mtv_reset_n")

/* MTV_INT */
#define SMS_IRQ_GPIO          MDTV_INT_N_GPIO

#define TX_BUFFER_SIZE			0x200

/* physical layer variables */
/*! global bus data */
struct spiphy_dev_s {
	struct completion transfer_in_process;
	void (*interruptHandler) (void *);
	void *intr_context;
	struct spi_device *dev;	/*!< device model stuff */
	int irq;

	char *txpad;
	dma_addr_t txpad_phy_addr;
};

struct omap2_mcspi_cs {
	void __iomem		*base;
	unsigned long		phys;
	int			word_len;
};


#define MOT_LP3907_DEV      1

#define INTERRUPT_DISABLE     0
#define INTERRUPT_ENABLE      1

static struct spi_device *smsmdtv_dev;

static int smsmdtv_int_enable_flag = INTERRUPT_ENABLE;

/* SPI interrupt handler */
static irqreturn_t spibus_interrupt(int irq, void *context)
{
	struct spiphy_dev_s *spiphy_dev = (struct spiphy_dev_s *) context;

	if (spiphy_dev->interruptHandler)
		spiphy_dev->interruptHandler(spiphy_dev->intr_context);

	return IRQ_HANDLED;
}

#define SPI_MIN_BYTES (4)
#define SMS_MAX_BUF_SIZE (0x1000)

#if defined(MOT_FEAT_OMAP_DMA_USE)
void smsspibus_xfer(void *context, unsigned char *txbuf,
		    unsigned long txbuf_phy_addr, unsigned char *rxbuf,
		    unsigned long rxbuf_phy_addr, int len)
{
  struct spi_transfer t;
  struct spi_message  m;
  struct spiphy_dev_s *spiphy_dev = (struct spiphy_dev_s *) context;
  unsigned long txdma;

  int status = -1;

  if (txbuf == 0) {
    txbuf = (unsigned char *)spiphy_dev->txpad;
    txdma  = spiphy_dev->txpad_phy_addr;
  } else {
    txdma = txbuf_phy_addr;
  }

  spi_message_init(&m);

  m.is_dma_mapped = 1;
  memset(&t, 0, sizeof(struct spi_transfer));

  t.tx_buf    = txbuf;
  t.tx_dma    = txdma;
  t.len       = len;
  t.rx_buf    = rxbuf;
  t.rx_dma    = rxbuf_phy_addr;
  t.delay_usecs = 1;

  spi_message_add_tail(&t, &m);

  status = spi_sync(spiphy_dev->dev, &m);
}

#else
void smsspibus_xfer(void *context, unsigned char *txbuf,
		    unsigned long txbuf_phy_addr, unsigned char *rxbuf,
		    unsigned long rxbuf_phy_addr, int len)
{
	struct spi_transfer	*t = 0;
	struct spi_message	 m;
	struct spiphy_dev_s *spiphy_dev = (struct spiphy_dev_s *) context;
	int i = 0;
	int status = -1;

	t = spiphy_dev->spi_xfer;

	spi_message_init(&m);

	for (i = 0; i < len/SPI_MIN_BYTES; i++) {
		memset(&t[i], 0, sizeof(struct spi_transfer));
		t[i].tx_buf	 = txbuf;
		t[i].len		 = SPI_MIN_BYTES;
		t[i].rx_buf	 = rxbuf;

		spi_message_add_tail(&t[i], &m);

		if (txbuf)
			txbuf += SPI_MIN_BYTES;
		rxbuf += SPI_MIN_BYTES;
	}

	status = spi_sync(spiphy_dev->dev, &m);

}
#endif

void smschipreset(void *context)
{
  return;
}

int smsmdtv_power_control(int pwrup_enable)
{
  int ret = 0, line = 0;

  tdmblog("power control = %d\n", pwrup_enable);

  if (pwrup_enable == 1) {

	ret = omap_cfg_reg(AC3_34XX_MDTV_SIMO_ON);
	ret = omap_cfg_reg(AD4_34XX_MDTV_SOMI_ON);
	ret = omap_cfg_reg(AD3_34XX_MDTV_CS_ON);
	ret = omap_cfg_reg(AA3_34XX_MDTV_CLK_ON);

	/* PWDN High */
	gpio_set_value(MDTV_PWDN_GPIO, 1);
	udelay(20);  /* at least, T = 10usec */

	/* Reset High */
	gpio_set_value(MDTV_RESET_N_GPIO, 1);
	udelay(20);  /* at least, T = 10usec */

	/* SMSMDTV interrupt enable */
	if (smsmdtv_int_enable_flag == INTERRUPT_DISABLE) {
	enable_irq(OMAP_GPIO_IRQ(SMS_IRQ_GPIO));
	smsmdtv_int_enable_flag = INTERRUPT_ENABLE;
	printk(KERN_INFO "enable_irq().\n");
	}
  } else {

    /* SMSMDTV interrupt disable */
    if (smsmdtv_int_enable_flag == INTERRUPT_ENABLE) {
      disable_irq(OMAP_GPIO_IRQ(SMS_IRQ_GPIO));
      smsmdtv_int_enable_flag = INTERRUPT_DISABLE;
      printk(KERN_INFO "disable_irq().\n");
    }

	/* Reset Low */
	gpio_set_value(MDTV_RESET_N_GPIO, 0);
	udelay(20);  /* at least, T = 10usec */

	/* PWDN Low */
	gpio_set_value(MDTV_PWDN_GPIO, 0);
	/*msleep(1);*/

	ret = omap_cfg_reg(AC3_34XX_MDTV_SIMO_OFF);
	ret = omap_cfg_reg(AD4_34XX_MDTV_SOMI_OFF);
	ret = omap_cfg_reg(AD3_34XX_MDTV_CS_OFF);
	ret = omap_cfg_reg(AA3_34XX_MDTV_CLK_OFF);
	}

  if (ret < 0)
    tdmblog("Power control sequence is failed.\n");

	return ret;
}

static int __devinit smsmdtv_probe(struct spi_device *spi)
{
/*
	struct spi_device *slave = spi;
	struct omap2_mcspi_cs *cs = \
		(struct omap2_mcspi_cs *)slave->controller_state;
*/
	int ret;

	/* SPI setup is needed */
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "[SMS] SPI setup failed\n");
		goto err;
	}

	smsmdtv_dev = spi;
    return ret;
err:
	return -1;
}

static struct spi_driver smsmdtv_driver = {
  .driver = {
    .name = "smsmdtv",
    .bus  = &spi_bus_type,
    .owner = THIS_MODULE,
  },
  .probe = smsmdtv_probe,
};

void *smsspiphy_init(void *context,
	void (*smsspi_interruptHandler) (void *), void *intr_context)
{
  struct spiphy_dev_s *spiphy_dev;
  int ret;

  smsmdtv_dev = 0;

	spiphy_dev = kmalloc(sizeof(struct spiphy_dev_s), GFP_KERNEL);
	if (spiphy_dev == 0) {
		sms_err("malloc for spi_dev failed");
		goto err_malloc;
	}
	spiphy_dev->interruptHandler = smsspi_interruptHandler;
	spiphy_dev->intr_context     = intr_context;

  ret = spi_register_driver(&smsmdtv_driver);

	if (ret < 0 || smsmdtv_dev == 0) {
		sms_info("Cann't get SPI device\n");
		goto err_register;
  }

	spiphy_dev->dev      = smsmdtv_dev;


  /*spi_loop_test(slave);*/

  spiphy_dev->irq = OMAP_GPIO_IRQ(SMS_IRQ_GPIO);

  set_irq_type(spiphy_dev->irq, IRQ_TYPE_EDGE_RISING);
	ret = request_irq(spiphy_dev->irq, spibus_interrupt, \
		IRQF_TRIGGER_RISING|IRQF_DISABLED, "smsmdtv", spiphy_dev);

	if (ret < 0) {
		sms_info("Unable to request irq %d", ret);
		goto err_irq;
	}

	/* interrupt disable */
	disable_irq(OMAP_GPIO_IRQ(SMS_IRQ_GPIO));
	smsmdtv_int_enable_flag = INTERRUPT_DISABLE;
	printk(KERN_INFO "smsspiphy_init(): disable_irq\n");

	spiphy_dev->txpad = dma_alloc_coherent(NULL, TX_BUFFER_SIZE,
			&spiphy_dev->txpad_phy_addr,
			GFP_KERNEL|GFP_DMA);
	if (!spiphy_dev->txpad) {
    ret = -ENOMEM;
    goto err_txpad;
	}
  memset(spiphy_dev->txpad, 0xFF, TX_BUFFER_SIZE);


	PDEBUG("exiting\n");
	return spiphy_dev;

err_txpad:
  free_irq(spiphy_dev->irq, spiphy_dev);
err_irq:
  gpio_free(SMS_IRQ_GPIO);
  spi_unregister_driver(&smsmdtv_driver);
err_register:
  kfree(spiphy_dev);
err_malloc:
	return 0;
}

int smsspiphy_deinit(void *context)
{
	struct spiphy_dev_s *spiphy_dev = (struct spiphy_dev_s *) context;

	gpio_free(SMS_IRQ_GPIO);
  spi_unregister_driver(&smsmdtv_driver);
	free_irq(spiphy_dev->irq, spiphy_dev);
  dma_free_coherent(NULL, TX_BUFFER_SIZE, \
	spiphy_dev->txpad, spiphy_dev->txpad_phy_addr);
	kfree(spiphy_dev);

	return 0;
}

void smsspiphy_set_config(struct spiphy_dev_s *spiphy_dev, int clock_divider)
{
	/*Omap_SPI_Enable(&(spiphy_dev->sspdev));*/
}

void prepareForFWDnl(void *context)
{
	struct spiphy_dev_s *spiphy_dev = (struct spiphy_dev_s *) context;
	smsspiphy_set_config(spiphy_dev, 2);
	msleep(100);
}

void fwDnlComplete(void *context, int App)
{
	struct spiphy_dev_s *spiphy_dev = (struct spiphy_dev_s *) context;
	smsspiphy_set_config(spiphy_dev, 1);
	msleep(100);
}

