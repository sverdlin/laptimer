/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2020 Alexander Sverdlin
 */

#ifndef _USB_H_
#define _USB_H_

/* PID has been officially sub-licensed from Microchip Inc. */
#define USB_VID			0x04D8
#define USB_PID			0xEB86

#define USB_MANUFACTURER	u"SV Sulmetingen e.V. Abteilung Motorsport"
#define USB_PRODUCT		u"Laptimer"
#define USB_PRODUCT_RELEASE	0x0001
#define USB_POWER_MA		(10 + 3 * 10 + 20)

#define USB_PLL_DIV (F_CPU / 8000000)
#define USB_PLL_CFG (((USB_PLL_DIV % 5 == 0) ? BIT(PLLP2) : 0) | \
		     ((USB_PLL_DIV % 3 == 0) ? BIT(PLLP1) : 0) | \
		     ((USB_PLL_DIV % 2 == 0) ? BIT(PLLP0) : 0))

void usb_cdc_init(uint8_t plldiv, bool reg3v3en);

extern volatile uint8_t usb_config;
#define usb_cdc_configured() (usb_config)

int8_t usb_cdc_drain(void);
int8_t usb_cdc_putchar(char c);

#endif /* _USB_H_ */
