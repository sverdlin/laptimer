/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2020 Alexander Sverdlin
 */

#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "misc.h"
#include "usb.h"

#define EP_SIZE(s) ((s == 64) ? (BIT(EPSIZE0) | BIT(EPSIZE1)) : \
		    (s == 32) ? BIT(EPSIZE1) : \
		    (s == 16) ? BIT(EPSIZE0) : 0)
#define EP_BANKS(b) ((b == 2) ? BIT(EPBK0) : 0)

/* Control EP request types */
#define GET_STATUS			0
#define CLEAR_FEATURE			1
#define SET_FEATURE			3
#define SET_ADDRESS			5
#define GET_DESCRIPTOR			6
#define GET_CONFIGURATION		8
#define SET_CONFIGURATION		9
#define GET_INTERFACE			10
#define SET_INTERFACE			11

#define CDC_SET_LINE_CODING		0x20
#define CDC_GET_LINE_CODING		0x21
#define CDC_SET_CONTROL_LINE_STATE	0x22

#define USB_ENDPOINT_HALT		0x00

#define TRANSMIT_FLUSH_TIMEOUT	0	/* Value in frame counts (1ms), 0 == disabled */

#define TRANSMIT_TIMEOUT	5	/* Value in frame counts (1ms) */

#define USB_ENDPOINT0_SIZE	16

#define CDC_ACM_ENDPOINT	2
#define CDC_ACM_SIZE		16
/* Interval for polling endpoint data transfers. Value in frame counts (1ms). */
#define CDC_ACM_INTERVAL	64

#define CDC_RX_ENDPOINT		3
#define CDC_RX_SIZE		32

#define CDC_TX_ENDPOINT		4
#define CDC_TX_SIZE		32

#define USB_MAX_ENDPOINT	4

volatile uint8_t usb_config;
volatile static uint8_t tx_flush_tmr;

struct ep_config_entry {
	uint8_t ep;
	uint8_t cfg0;
	uint8_t cfg1;
};

/* AND the masks */
#define EP_DIR_IN	0xC1
#define EP_DIR_OUT	0xC0
#define EP_CONTROL	0x01
#define EP_ISO		0x41
#define EP_BULK		0x81
#define EP_INT		0xC1

static const __flash struct ep_config_entry ep_config_tab[] = {
	{
		.ep	= CDC_ACM_ENDPOINT,
		.cfg0	= EP_INT & EP_DIR_IN,
		.cfg1	= EP_SIZE(CDC_ACM_SIZE) | EP_BANKS(1) | BIT(ALLOC),
	},
	{
		.ep	= CDC_RX_ENDPOINT,
		.cfg0	= EP_BULK & EP_DIR_OUT,
		.cfg1	= EP_SIZE(CDC_RX_SIZE) | EP_BANKS(2) | BIT(ALLOC),
	},
	{
		.ep	= CDC_TX_ENDPOINT,
		.cfg0	= EP_BULK & EP_DIR_IN,
		.cfg1	= EP_SIZE(CDC_TX_SIZE) | EP_BANKS(2) | BIT(ALLOC),
	},
};

struct usb_device_descriptor {
	uint8_t		bLength;
	uint8_t		bDescriptorType;
	uint16_t	bcdUSB;
	uint8_t		bDeviceClass;
	uint8_t		bDeviceSubClass;
	uint8_t		bDeviceProtocol;
	uint8_t		bMaxPacketSize0;
	uint16_t	idVendor;
	uint16_t	idProduct;
	uint16_t	bcdDevice;
	uint8_t		iManufacturer;
	uint8_t		iProduct;
	uint8_t		iSerialNumber;
	uint8_t		bNumConfigurations;
} __attribute__((packed));

const __flash struct usb_device_descriptor usb_device_descriptor = {
	.bLength		= sizeof(struct usb_device_descriptor),
	.bDescriptorType	= 1,
	.bcdUSB			= cpu_to_le16(0x200),
	.bDeviceClass		= 2,
	.bMaxPacketSize0	= USB_ENDPOINT0_SIZE,
	.idVendor		= USB_VID,
	.idProduct		= USB_PID,
	.bcdDevice		= cpu_to_le16(USB_PRODUCT_RELEASE),
	.iManufacturer		= 1,
	.iProduct		= 2,
	.bNumConfigurations	= 1,
};

/* OR the masks */
#define ATTR_DFLT	0x80
#define ATTR_SELF_PWR	0x40
#define ATTR_REM_WAKEUP	0x20

struct usb_config_descriptor {
	uint8_t		bLength;
	uint8_t		bDescriptorType;	/* Configuration Descriptor (0x02) */
	uint16_t	wTotalLength;		/* Total length in bytes of data returned */
	uint8_t		bNumInterfaces;
	uint8_t		bConfigurationValue;	/* Value to use as an argument to select this configuration */
	uint8_t		iConfiguration;		/* Index of String Descriptor describing this configuration */
	uint8_t		bmAttributes;
	uint8_t		bMaxPower;		/* Maximum Power Consumption in 2mA units */
} __attribute__((packed));

struct usb_int_descriptor {
	uint8_t		bLength;
	uint8_t		bDescriptorType;	/* Interface Descriptor (0x04) */
	uint8_t		bInterfaceNumber;
	uint8_t		bAlternateSetting;	/* Value used to select alternative setting */
	uint8_t		bNumEndpoints;
	uint8_t		bInterfaceClass;
	uint8_t		bInterfaceSubClass;
	uint8_t		bInterfaceProtocol;
	uint8_t		iInterface;		/* Index of String Descriptor Describing this interface */
} __attribute__((packed));

/* AND the masks to obtain the capabilities */
#define CM_COMM_CLASS	1
#define CM_DATA_CLASS	3
#define CM_CALL_HANDLE	1
#define CM_NO_CALL_HND	2

struct usb_call_mgmt_descriptor {
	uint8_t		bFunctionLength;
	uint8_t		bDescriptorType;	/* CS_INTERFACE */
	uint8_t		bDescriptorSubtype;
	uint8_t		bmCapabilities;
	uint8_t		bDataInterface;		/* Number Interface number of Data Class interface optionally used for call management */
} __attribute__((packed));

/* OR the masks */
#define ACM_NETWORK_CONN	0x08
#define ACM_SEND_BREAK		0x04
/* Device supports the request combination of Set_Line_Coding, Set_Control_Line_State, Get_Line_Coding and the notification Serial_State */
#define ACM_LINE		0x02
/* Device supports the request combination of Set_Comm_Feature, Clear_Comm_Feature, and Get_Comm_Feature */
#define ACM_COMM		0x01

struct usb_acm_descriptor {
	uint8_t		bFunctionLength;
	uint8_t		bDescriptorType;	/* CS_INTERFACE */
	uint8_t		bDescriptorSubtype;
	uint8_t		bmCapabilities;
} __attribute__((packed));

/* OR with address */
#define EP_ADDR_IN	0x80
#define EP_ADDR_OUT	0x00

/* AND the masks */
#define EP_ATTR_CTRL	0x00
#define EP_ATTR_ISO	0x3D
#define EP_ATTR_BULK	0x02
#define EP_ATTR_INT	0x03
/* Only if Isochronous EP */
#define EP_NO_SYNC	0x33
#define EP_ASYNC	0x37
#define EP_ADAPTIVE	0x3B
#define EP_SYNCHRONOUS	0x3F
#define EP_DATA		0x0F
#define EP_FEEDBACK	0x1F
#define EP_EXPLICIT	0x2F

struct usb_ep_descriptor {
	uint8_t		bLength;
	uint8_t		bDescriptorType;
	uint8_t		bEndpointAddress;
	uint8_t		bmAttributes;
	uint16_t	wMaxPacketSize;
	/*
	 * Interval for polling endpoint data transfers. Value in frame counts.
	 * Ignored for Bulk & Control Endpoints.
	 * Isochronous must equal 1 and field may range from 1 to 255 for interrupt endpoints.
	 */
	uint8_t		bInterval;
} __attribute__((packed));

struct usb_union_descriptor {
	uint8_t		bFunctionLength;
	uint8_t		bDescriptorType;	/* Constant CS_INTERFACE */
	uint8_t		bDescriptorSubtype;
	uint8_t		bMasterInterface;
	uint8_t		bSlaveInterface0;
} __attribute__((packed));

#define CS_INTERFACE	0x24

struct usb_cdc_hdr_descriptor {
	uint8_t		bFunctionLength;
	uint8_t		bDescriptorType;	/* CS_INTERFACE descriptor type */
	uint8_t		bDescriptorSubtype;
	uint16_t	bcdCDC;			/* CDC Specification release number in binary-coded decimal */
} __attribute__((packed));

static const __flash struct usb_config1_descriptor {
	struct usb_config_descriptor	cd;
	struct usb_int_descriptor	id1;
	struct usb_cdc_hdr_descriptor	hd;
	struct usb_call_mgmt_descriptor	cm;
	struct usb_acm_descriptor	acm;
	struct usb_union_descriptor	ud;
	struct usb_ep_descriptor	acm_ep;
	struct usb_int_descriptor	id2;
	struct usb_ep_descriptor	rx_ep;
	struct usb_ep_descriptor	tx_ep;
} __attribute__((packed)) usb_config1_descriptor = {
	.cd = {
		.bLength		= sizeof(struct usb_config_descriptor),
		.bDescriptorType	= 0x02,
		.wTotalLength		= cpu_to_le16(sizeof(usb_config1_descriptor)),
		.bNumInterfaces		= 2,
		.bConfigurationValue	= 1,
		.bmAttributes		= ATTR_DFLT,
		.bMaxPower		= USB_POWER_MA / 2,
	},
	.id1 = {
		.bLength		= sizeof(struct usb_int_descriptor),
		.bDescriptorType	= 0x04,
		.bInterfaceNumber	= 0,
		.bNumEndpoints		= 1,
		.bInterfaceClass	= 0x02,
		.bInterfaceSubClass	= 0x02,
		.bInterfaceProtocol	= 0x01,
	},
	.hd = {
		.bFunctionLength	= sizeof(struct usb_cdc_hdr_descriptor),
		.bDescriptorType	= CS_INTERFACE,
		.bcdCDC			= cpu_to_le16(0x0110),
	},
	.cm = {
		.bFunctionLength	= sizeof(struct usb_call_mgmt_descriptor),
		.bDescriptorType	= CS_INTERFACE,
		.bDescriptorSubtype	= 0x01,
		.bmCapabilities		= CM_CALL_HANDLE,
		.bDataInterface		= 1,
	},
	.acm = {
		.bFunctionLength	= sizeof(struct usb_acm_descriptor),
		.bDescriptorType	= CS_INTERFACE,
		.bDescriptorSubtype	= 0x02,
		.bmCapabilities		= ACM_SEND_BREAK | ACM_LINE,
	},
	.ud = {
		.bFunctionLength	= sizeof(struct usb_union_descriptor),
		.bDescriptorType	= CS_INTERFACE,
		.bDescriptorSubtype	= 0x06,
		.bMasterInterface	= 0,
		.bSlaveInterface0	= 1,
	},
	.acm_ep = {
		.bLength		= sizeof(struct usb_ep_descriptor),
		.bDescriptorType	= 0x05,
		.bEndpointAddress	= CDC_ACM_ENDPOINT | EP_ADDR_IN,
		.bmAttributes		= EP_ATTR_INT,
		.wMaxPacketSize		= cpu_to_le16(CDC_ACM_SIZE),
		.bInterval		= CDC_ACM_INTERVAL,
	},
	.id2 = {
		.bLength		= sizeof(struct usb_int_descriptor),
		.bDescriptorType	= 0x04,
		.bInterfaceNumber	= 1,
		.bNumEndpoints		= 2,
		.bInterfaceClass	= 0x0A,
		.bInterfaceSubClass	= 0x00,
		.bInterfaceProtocol	= 0x00,
	},
	.rx_ep = {
		.bLength		= sizeof(struct usb_ep_descriptor),
		.bDescriptorType	= 0x05,
		.bEndpointAddress	= CDC_RX_ENDPOINT | EP_ADDR_OUT,
		.bmAttributes		= EP_ATTR_BULK,
		.wMaxPacketSize		= cpu_to_le16(CDC_RX_SIZE),
	},
	.tx_ep = {
		.bLength		= sizeof(struct usb_ep_descriptor),
		.bDescriptorType	= 0x05,
		.bEndpointAddress	= CDC_TX_ENDPOINT | EP_ADDR_IN,
		.bmAttributes		= EP_ATTR_BULK,
		.wMaxPacketSize		= cpu_to_le16(CDC_TX_SIZE),
	},
};

#define USB_STRING(sym, str)					\
static const __flash struct usb_string_descriptor_##sym {	\
	uint8_t bLength;					\
	uint8_t bDescriptorType;				\
	uint16_t wString[sizeof(str) / 2 - 1];			\
} __attribute__((packed)) sym = {				\
	.bLength		= sizeof(sym),			\
	.bDescriptorType	= 3,				\
	.wString = str,						\
}

/* String Index 0 should return a list of supported languages. */
USB_STRING(usb_string0, u"\x0409");
USB_STRING(usb_string1, USB_MANUFACTURER);
USB_STRING(usb_string2, USB_PRODUCT);

struct usb_descriptor_entry {
	uint16_t wValue;
	uint16_t wIndex;
	const __flash uint8_t *desc;
	uint8_t len;
} __attribute__((packed));

#define USB_DESCRIPTOR(_wValue, _wIndex, _desc)		\
{							\
	.wValue	= _wValue,				\
	.wIndex	= _wIndex,				\
	.desc	= (const __flash uint8_t *)&_desc,	\
	.len	= sizeof(_desc),			\
}

static const __flash struct usb_descriptor_entry usb_descriptor_tab[] = {
	USB_DESCRIPTOR(0x0100, 0x0000, usb_device_descriptor),
	USB_DESCRIPTOR(0x0200, 0x0000, usb_config1_descriptor),
	USB_DESCRIPTOR(0x0300, 0x0000, usb_string0),
	USB_DESCRIPTOR(0x0301, 0x0409, usb_string1),
	USB_DESCRIPTOR(0x0302, 0x0409, usb_string2),
};

/* AND the following bit masks to obtain bmRequestType value */
#define RT_HOST2DEV	0x7F
#define RT_DEV2HOST	0xFF
#define RT_STD		0x9F
#define RT_CLASS	0xBF
#define RT_VENDOR	0xDF
#define RT_DEVICE	0xE0
#define RT_INTERFACE	0xE1
#define RT_ENDPOINT	0xE2

struct usb_setup_packet {
	uint8_t		bmRequestType;
	uint8_t		bRequest;
	uint16_t	wValue;
	uint16_t	wIndex;
	uint16_t	wLength;
} __attribute__((packed));

void usb_cdc_init(uint8_t pllcfg, bool reg3v3en)
{
	REGCR = BIT(REGDIS) * !reg3v3en;

	/*
	 * Enable USB controller and D+ D-, but freeze USB clock.
	 * For some reason nothing works if USB is not enabled here, prior to
	 * the rest initialization
	 */
	USBCON = BIT(FRZCLK) | BIT(USBE);

	PLLCSR &= ~BIT(PLLE);
	/* Configure and start PLL */
	PLLCSR = (BIT(PLLE) | pllcfg) & (pllcfg | ~(BIT(PLLP2) | BIT(PLLP1) | BIT(PLLP0)));
	while (!(PLLCSR & BIT(PLOCK)));		/* Wait for PLL to lock */

	USBCON = BIT(USBE);			/* Unfreeze USB clock */

	UDCON &= ~BIT(DETACH);			/* Bus attach */
	/* End Of Reset + Start Of Frame Interrupts */
	UDIEN = BIT(EORSTE) |
		(BIT(SOFE) * !!TRANSMIT_FLUSH_TIMEOUT);
}

static void usb_reset_tx_tmr(void)
{
	if (!TRANSMIT_FLUSH_TIMEOUT)
		return;
	tx_flush_tmr = 0;
}

#ifndef KILLBK
#define KILLBK RXOUTI
#endif

static void usb_flush_cur_ep(void)
{
	UEINTX = (uint8_t)~BIT(FIFOCON) & ~BIT(KILLBK) & ~BIT(TXINI);
}

/* Shall be called with interrupts disabled */
static void usb_flush_ep(uint8_t ep)
{
	UENUM = ep;
	usb_flush_cur_ep();
}

int8_t usb_cdc_drain(void)
{
	if (!usb_cdc_configured())
		return -1;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		usb_flush_ep(CDC_TX_ENDPOINT);

	return 0;
}

static bool usb_cur_txbuf_avail(void)
{
	return UEINTX & BIT(RWAL);
}

static bool usb_txbuf_avail(uint8_t ep)
{
	bool ret;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		UENUM = ep;
		ret = usb_cur_txbuf_avail();
	}
	return ret;
}

/* Call with interrupts disabled */
static bool usb_get_stall(uint8_t ep)
{
	bool ret;

	UENUM = ep;
	ret = UECONX & BIT(STALLRQ);
	UENUM = 0;
	return ret;
}

int8_t usb_cdc_putchar(char c)
{
	uint8_t timeout = UDFNUML + TRANSMIT_TIMEOUT;

	for (;;) {
		if (!usb_cdc_configured())
			return -1;
		if (usb_txbuf_avail(CDC_TX_ENDPOINT))
			break;
		if (timeout == UDFNUML)
			return -1;
	}

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		UENUM	= CDC_TX_ENDPOINT;
		UEDATX	= c;
		if (!usb_cur_txbuf_avail())
			usb_flush_cur_ep();
	}

	return 0;
}

static void usb_process_tx_timeout(void)
{
	if (!TRANSMIT_FLUSH_TIMEOUT || !usb_cdc_configured())
		return;
	if (!tx_flush_tmr || --tx_flush_tmr)
		return;
	usb_flush_ep(CDC_TX_ENDPOINT);
}

/* Device level events */
ISR(USB_GEN_vect)
{
	uint8_t mask = UDINT;

	UDINT = 0;

	if (unlikely(mask & BIT(EORSTI))) {
		usb_config = 0;

		UENUM	= 0;						/* EP0 */
		UECONX	= BIT(EPEN);					/* Enable */
		UECFG1X	= EP_SIZE(USB_ENDPOINT0_SIZE) | EP_BANKS(1) | BIT(ALLOC);
		UEIENX	= BIT(RXSTPE) | BIT(RXOUTE);	/* Enable EP Interrupts */
	}

	if ((mask & BIT(SOFI)) && !!TRANSMIT_FLUSH_TIMEOUT)
		usb_process_tx_timeout();
}

static void usb_send_in(void)
{
	UEINTX = ~BIT(TXINI);
}

static void usb_read_data(uint8_t *buf, uint8_t count)
{
	while (count--)
		*buf++ = UEDATX;
}

static void usb_write_data(const uint8_t *buf, uint8_t count)
{
	while (count--)
		UEDATX = *buf++;
}

static void usb_stall_ep(void)
{
	UECONX = BIT(STALLRQ) | BIT (EPEN);
}

static void usb_prepare_to_send(void)
{
	UEIENX |= BIT(TXINE);	/* Enable TXINI interrupt */
}

/* Call with interrupts disabled */
static void usb_load_ep_config(void)
{
	uint8_t i;

	for (i = 0; i < ARRAY_SIZE(ep_config_tab); i++) {
		UENUM = ep_config_tab[i].ep;
		UECONX = BIT(EPEN);
		UECFG0X = ep_config_tab[i].cfg0;
		UECFG1X = ep_config_tab[i].cfg1;
	}

	/* Reset all but control endpoints */
	UERST = 0x1E;
	UERST = 0;
}

/* Call with interrupts disabled */
static void usb_clr_stall(uint8_t ep)
{
	UENUM = ep;
	UECONX = BIT(EPEN) | BIT(STALLRQC) | BIT(RSTDT);

	UERST = BIT(ep);
	UERST = 0;
}

/* Call with interrupts disabled */
static void usb_set_stall(uint8_t ep)
{
	UENUM = ep;
	UECONX = BIT(EPEN) | BIT(STALLRQ);
}

struct line_coding {
	uint32_t	dwDTERate;
	uint8_t		bCharFormat;
	uint8_t		bParityType;
	uint8_t		bDataBits;
} __attribute__((packed));

ISR(USB_COM_vect)
{
	uint8_t i;
	uint8_t mask;
	static struct usb_setup_packet sp;
	static const __flash uint8_t *txptr;
	static struct line_coding line_coding = {
		.dwDTERate = cpu_to_le32(9600),
		.bDataBits = 8,
	};

	UENUM = 0;
	mask = UEINTX;
	if (likely(mask & BIT(RXSTPI))) {
		usb_read_data((uint8_t *)&sp, sizeof(sp));
		sp.wValue = le16_to_cpu(sp.wValue);
		sp.wIndex = le16_to_cpu(sp.wIndex);
		sp.wLength = le16_to_cpu(sp.wLength);
		UEINTX = ~BIT(RXSTPI);	/* ACK the interrupt */

		switch (sp.bRequest) {
		case GET_DESCRIPTOR:
			for (i = 0; i < ARRAY_SIZE(usb_descriptor_tab); i++)
				if (unlikely(usb_descriptor_tab[i].wValue == sp.wValue &&
					     usb_descriptor_tab[i].wIndex == sp.wIndex)) {
					txptr = usb_descriptor_tab[i].desc;
					if (likely(usb_descriptor_tab[i].len < sp.wLength))
						sp.wLength = usb_descriptor_tab[i].len;
					usb_prepare_to_send();
					return;
				}
			break;

		case SET_ADDRESS:
			UDADDR = 0;
			sp.wValue &= ~BIT(ADDEN);
			usb_prepare_to_send();
			return;

		case SET_CONFIGURATION:
			if (unlikely(sp.bmRequestType != (RT_DEVICE & RT_STD & RT_HOST2DEV)))
				break;
			usb_prepare_to_send();
			return;

		case GET_CONFIGURATION:
			if (unlikely(sp.bmRequestType != (RT_DEVICE & RT_STD & RT_DEV2HOST)))
				break;
			usb_prepare_to_send();
			return;

		case CDC_SET_LINE_CODING:
			if (unlikely(sp.bmRequestType != (RT_INTERFACE & RT_CLASS & RT_HOST2DEV)))
				break;
			return;

		case CDC_GET_LINE_CODING:
			if (unlikely(sp.bmRequestType != (RT_INTERFACE & RT_CLASS & RT_DEV2HOST)))
				break;
			usb_prepare_to_send();
			return;

		case CDC_SET_CONTROL_LINE_STATE:
			if (unlikely(sp.bmRequestType != (RT_INTERFACE & RT_CLASS & RT_HOST2DEV)))
				break;
			usb_prepare_to_send();
			return;

		case GET_STATUS:
			if (unlikely(sp.bmRequestType != (RT_ENDPOINT & RT_STD & RT_DEV2HOST) &&
				     sp.bmRequestType != (RT_DEVICE & RT_STD & RT_DEV2HOST)))
				break;
			usb_prepare_to_send();
			return;

		case CLEAR_FEATURE:
		case SET_FEATURE:
			sp.wIndex &= 0xF;
			if (unlikely(sp.bmRequestType != (RT_ENDPOINT & RT_STD & RT_HOST2DEV) ||
				     sp.wValue != USB_ENDPOINT_HALT ||
				     !(uint8_t)sp.wIndex ||
				     (uint8_t)sp.wIndex > USB_MAX_ENDPOINT))
				break;
			usb_prepare_to_send();
			return;
		}
	}

	if (likely(mask & BIT(RXOUTI))) {
		switch (sp.bRequest) {
		case CDC_SET_LINE_CODING:
			usb_read_data((uint8_t *)&line_coding, sizeof(line_coding));
			usb_prepare_to_send();
			break;
			break;
		}

		UEINTX = ~BIT(RXOUTI);	/* ACK the interrupt */

		switch (sp.bRequest) {
		case CDC_SET_LINE_CODING:
			return;
		}
	}

	if (likely(UEIENX & mask & BIT(TXINI))) {
		UEIENX &= ~BIT(TXINE);	/* Disable interrupt */

		switch (sp.bRequest) {
		case GET_DESCRIPTOR:
			for (i = 0; sp.wLength && i < USB_ENDPOINT0_SIZE; i++) {
				UEDATX = *txptr++;
				sp.wLength--;
			}
			usb_send_in();
			if (sp.wLength)
				usb_prepare_to_send();
			return;

		case SET_ADDRESS:
			if (UDADDR) {
				/* ACK packet has been sent, switch the address */
				UDADDR |= BIT(ADDEN);
			} else {
				UDADDR = sp.wValue;
				/* ACK with empty IN packet */
				usb_send_in();
				usb_prepare_to_send();
			}
			return;

		case SET_CONFIGURATION:
			usb_send_in();
			usb_load_ep_config();
			usb_config = sp.wValue;
			usb_reset_tx_tmr();
			return;

		case GET_CONFIGURATION:
			UEDATX = usb_config;
			usb_send_in();
			return;

		case CDC_SET_LINE_CODING:
		case CDC_SET_CONTROL_LINE_STATE:
			usb_send_in();
			return;

		case CDC_GET_LINE_CODING:
			usb_write_data((uint8_t *)&line_coding, sizeof(line_coding));
			usb_send_in();
			return;

		case GET_STATUS:
			if (sp.bmRequestType == (RT_ENDPOINT & RT_STD & RT_DEV2HOST)) {
				/* Bit0 of 16-bit reply == 1 if Endpoint[wIndex] has been halted */
				sp.wIndex = cpu_to_le16(usb_get_stall(sp.wIndex));
			} else {
				/* Device status, bit0 == 1 if self-powered device */
				sp.wIndex = cpu_to_le16(USB_POWER_MA ? 0 : 1);
			}
			usb_write_data((uint8_t *)&sp.wIndex, sizeof(sp.wIndex));
			usb_send_in();
			return;

		case CLEAR_FEATURE:
			usb_send_in();
			usb_clr_stall(sp.wIndex);
			return;

		case SET_FEATURE:
			usb_send_in();
			usb_set_stall(sp.wIndex);
			return;
		}
	}

	usb_stall_ep();
}
