#ifndef __LINUX_USB_OMAP_H
#define __LINUX_USB_OMAP_H

enum omap_usb_port_mode {
	/* ULPI_BYPASS = 0 */
	OMAP_USB_PORT_MODE_ULPI_PHY,

	/* ULPI_BYPASS = 1, CHANMODE = 1 */
	OMAP_USB_PORT_MODE_UTMI_PHY_6PIN,	/* FSLSMODE = 0x0 */
	OMAP_USB_PORT_MODE_UTMI_PHY_6PIN_ALT,	/* FSLSMODE = 0x1 */
	OMAP_USB_PORT_MODE_UTMI_PHY_3PIN,	/* FSLSMODE = 0x2 */
	OMAP_USB_PORT_MODE_UTMI_PHY_4PIN,	/* FSLSMODE = 0x3 */
	OMAP_USB_PORT_MODE_UTMI_TLL_6PIN,	/* FSLSMODE = 0x4 */
	OMAP_USB_PORT_MODE_UTMI_TLL_6PIN_ALT,	/* FSLSMODE = 0x5 */
	OMAP_USB_PORT_MODE_UTMI_TLL_3PIN,	/* FSLSMODE = 0x6 */
	OMAP_USB_PORT_MODE_UTMI_TLL_4PIN,	/* FSLSMODE = 0x7 */
	OMAP_USB_PORT_MODE_UTMI_TLL_2PIN,	/* FSLSMODE = 0xA */
	OMAP_USB_PORT_MODE_UTMI_TLL_2PIN_ALT,	/* FSLbSMODE = 0xB */

	/* ULPI_BYPASS = 1, CHANMODE = 0 */
	OMAP_USB_PORT_MODE_ULPI_TLL_SDR,	/* ULPIDDRMODE = 0 */
	OMAP_USB_PORT_MODE_ULPI_TLL_DDR,	/* ULPIDDRMODE = 1 */

};



#define OMAP_USB_PORT_FLAG_ENABLED	(1<<0)
#define OMAP_USB_PORT_FLAG_NOBITSTUFF	(1<<1)
#define OMAP_USB_PORT_FLAG_AUTOIDLE	(1<<2)

struct omap_usb_port_data {
	enum omap_usb_port_mode		mode;

	u32	flags;

	int	reset_delay;

	int	(*startup)(struct platform_device *dev, int port);
	void	(*shutdown)(struct platform_device *dev, int port);
	void	(*reset)(struct platform_device *dev, int port, int reset);
	void	(*suspend)(struct platform_device *dev, int port, int suspend);
};

#define OMAP_USB_FLAG_VBUS_INTERNAL_CHARGEPUMP	(1<<0)

struct omap_usb_platform_data {
	u32	flags;
	struct omap_usb_port_data	*port_data;
	int	num_ports;
};


#endif /* __LINUX_USB_OMAP_H */
