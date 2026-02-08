/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2001
 * Denis Peter, MPL AG Switzerland
 *
 * Note: Part of this code has been derived from linux
 */
#ifndef _USB_DEFS_H_
#define _USB_DEFS_H_

/* USB constants */

/* Device and/or Interface Class codes */
#define USB_CLASS_PER_INTERFACE  0	/* for DeviceClass */
#define USB_CLASS_AUDIO          1
#define USB_CLASS_COMM           2
#define USB_CLASS_HID            3
#define USB_CLASS_PRINTER	       7
#define USB_CLASS_MASS_STORAGE   8
#define USB_CLASS_HUB            9
#define USB_CLASS_DATA           10
#define USB_CLASS_VENDOR_SPEC    0xff

/* some HID sub classes */
#define USB_SUB_HID_NONE        0
#define USB_SUB_HID_BOOT        1

/* some UID Protocols */
#define USB_PROT_HID_NONE       0
#define USB_PROT_HID_KEYBOARD   1
#define USB_PROT_HID_MOUSE      2

/* Sub STORAGE Classes */
#define US_SC_RBC              1		/* Typically, flash devices */
#define US_SC_8020             2		/* CD-ROM */
#define US_SC_QIC              3		/* QIC-157 Tapes */
#define US_SC_UFI              4		/* Floppy */
#define US_SC_8070             5		/* Removable media */
#define US_SC_SCSI             6		/* Transparent */
#define US_SC_MIN              US_SC_RBC
#define US_SC_MAX              US_SC_SCSI

/* STORAGE Protocols */
#define US_PR_CB               1		/* Control/Bulk w/o interrupt */
#define US_PR_CBI              0		/* Control/Bulk/Interrupt */
#define US_PR_BULK             0x50		/* bulk only */

/* USB types */
#define USB_TYPE_STANDARD   (0x00 << 5)
#define USB_TYPE_CLASS      (0x01 << 5)
#define USB_TYPE_VENDOR     (0x02 << 5)
#define USB_TYPE_RESERVED   (0x03 << 5)

/* USB recipients */
#define USB_RECIP_DEVICE      0x00
#define USB_RECIP_INTERFACE   0x01
#define USB_RECIP_ENDPOINT    0x02
#define USB_RECIP_OTHER       0x03

/* USB directions */
#define USB_DIR_OUT           0
#define USB_DIR_IN            0x80

/*
 * bmRequestType: USB Device Requests, table 9.2 USB 2.0 spec.
 * (shifted) direction/type/recipient.
 */
#define DeviceRequest \
	((USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE) << 8)

#define DeviceOutRequest \
	((USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE) << 8)

#define InterfaceRequest \
	((USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_INTERFACE) << 8)

#define InterfaceOutRequest \
	((USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_INTERFACE) << 8)

#define EndpointRequest \
	((USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_INTERFACE) << 8)

#define EndpointOutRequest \
	((USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_INTERFACE) << 8)

/* class requests from the USB 2.0 hub spec, table 11-15 */
#define HUB_CLEAR_TT_BUFFER   8
#define HUB_RESET_TT          9
#define HUB_GET_TT_STATE      10
#define HUB_STOP_TT           11

#define HUB_CLASS_REQ(dir, type, request) ((((dir) | (type)) << 8) | (request))
/* GetBusState and SetHubDescriptor are optional, omitted */
#define ClearHubFeature		HUB_CLASS_REQ(USB_DIR_OUT, USB_RT_HUB, \
					      USB_REQ_CLEAR_FEATURE)
#define ClearPortFeature	HUB_CLASS_REQ(USB_DIR_OUT, USB_RT_PORT, \
					      USB_REQ_CLEAR_FEATURE)
#define GetHubDescriptor	HUB_CLASS_REQ(USB_DIR_IN, USB_RT_HUB, \
					      USB_REQ_GET_DESCRIPTOR)
#define GetHubStatus		HUB_CLASS_REQ(USB_DIR_IN, USB_RT_HUB, \
					      USB_REQ_GET_STATUS)
#define GetPortStatus		HUB_CLASS_REQ(USB_DIR_IN, USB_RT_PORT, \
					      USB_REQ_GET_STATUS)
#define SetHubFeature		HUB_CLASS_REQ(USB_DIR_OUT, USB_RT_HUB, \
					      USB_REQ_SET_FEATURE)
#define SetPortFeature		HUB_CLASS_REQ(USB_DIR_OUT, USB_RT_PORT, \
					      USB_REQ_SET_FEATURE)
#define ClearTTBuffer		HUB_CLASS_REQ(USB_DIR_OUT, USB_RT_PORT, \
					      HUB_CLEAR_TT_BUFFER)
#define ResetTT			HUB_CLASS_REQ(USB_DIR_OUT, USB_RT_PORT, \
					      HUB_RESET_TT)
#define GetTTState		HUB_CLASS_REQ(USB_DIR_IN, USB_RT_PORT, \
					      HUB_GET_TT_STATE)
#define StopTT			HUB_CLASS_REQ(USB_DIR_OUT, USB_RT_PORT, \
					      HUB_STOP_TT)

/* Descriptor types */
#define USB_DT_DEVICE        0x01
#define USB_DT_CONFIG        0x02
#define USB_DT_STRING        0x03
#define USB_DT_INTERFACE     0x04
#define USB_DT_ENDPOINT      0x05

#define USB_DT_HID          (USB_TYPE_CLASS | 0x01)
#define USB_DT_REPORT       (USB_TYPE_CLASS | 0x02)
#define USB_DT_PHYSICAL     (USB_TYPE_CLASS | 0x03)
#define USB_DT_HUB          (USB_TYPE_CLASS | 0x09)
#define USB_DT_SS_HUB       (USB_TYPE_CLASS | 0x0a)

/* Descriptor sizes per descriptor type */
#define USB_DT_DEVICE_SIZE      18
#define USB_DT_CONFIG_SIZE      9
#define USB_DT_INTERFACE_SIZE   9
#define USB_DT_ENDPOINT_SIZE    7
#define USB_DT_ENDPOINT_AUDIO_SIZE  9	/* Audio extension */
#define USB_DT_HUB_NONVAR_SIZE  7
#define USB_DT_HID_SIZE         9

/* Endpoints */
#define USB_ENDPOINT_NUMBER_MASK  0x0f	/* in bEndpointAddress */
#define USB_ENDPOINT_DIR_MASK     0x80

#define USB_ENDPOINT_XFERTYPE_MASK 0x03	/* in bmAttributes */
#define USB_ENDPOINT_XFER_CONTROL  0
#define USB_ENDPOINT_XFER_ISOC     1
#define USB_ENDPOINT_XFER_BULK     2
#define USB_ENDPOINT_XFER_INT      3

/* USB Packet IDs (PIDs) */
#define USB_PID_UNDEF_0             0xf0
#define USB_PID_OUT                 0xe1
#define USB_PID_ACK                 0xd2
#define USB_PID_DATA0               0xc3
#define USB_PID_UNDEF_4             0xb4
#define USB_PID_SOF                 0xa5
#define USB_PID_UNDEF_6             0x96
#define USB_PID_UNDEF_7             0x87
#define USB_PID_UNDEF_8             0x78
#define USB_PID_IN                  0x69
#define USB_PID_NAK                 0x5a
#define USB_PID_DATA1               0x4b
#define USB_PID_PREAMBLE            0x3c
#define USB_PID_SETUP               0x2d
#define USB_PID_STALL               0x1e
#define USB_PID_UNDEF_F             0x0f

/* Standard requests */
#define USB_REQ_GET_STATUS          0x00
#define USB_REQ_CLEAR_FEATURE       0x01
#define USB_REQ_SET_FEATURE         0x03
#define USB_REQ_SET_ADDRESS         0x05
#define USB_REQ_GET_DESCRIPTOR      0x06
#define USB_REQ_SET_DESCRIPTOR      0x07
#define USB_REQ_GET_CONFIGURATION   0x08
#define USB_REQ_SET_CONFIGURATION   0x09
#define USB_REQ_GET_INTERFACE       0x0A
#define USB_REQ_SET_INTERFACE       0x0B
#define USB_REQ_SYNCH_FRAME         0x0C

/* HID requests */
#define USB_REQ_GET_REPORT          0x01
#define USB_REQ_GET_IDLE            0x02
#define USB_REQ_GET_PROTOCOL        0x03
#define USB_REQ_SET_REPORT          0x09
#define USB_REQ_SET_IDLE            0x0A
#define USB_REQ_SET_PROTOCOL        0x0B

/* Device features */
#define USB_FEAT_HALT               0x00
#define USB_FEAT_WAKEUP             0x01
#define USB_FEAT_TEST               0x02

/* Test modes */
#define USB_TEST_MODE_J             0x01
#define USB_TEST_MODE_K             0x02
#define USB_TEST_MODE_SE0_NAK       0x03
#define USB_TEST_MODE_PACKET        0x04
#define USB_TEST_MODE_FORCE_ENABLE  0x05

/*************************************************************************
 * Hub defines
 */

/*
 * Hub request types
 */

#define USB_RT_HUB	(USB_TYPE_CLASS | USB_RECIP_DEVICE)
#define USB_RT_PORT	(USB_TYPE_CLASS | USB_RECIP_OTHER)

/*
 * Hub Class feature numbers
 */
#define C_HUB_LOCAL_POWER   0
#define C_HUB_OVER_CURRENT  1

/*
 * Port feature numbers
 */
#define USB_PORT_FEAT_CONNECTION     0
#define USB_PORT_FEAT_ENABLE         1
#define USB_PORT_FEAT_SUSPEND        2
#define USB_PORT_FEAT_OVER_CURRENT   3
#define USB_PORT_FEAT_RESET          4
#define USB_PORT_FEAT_POWER          8
#define USB_PORT_FEAT_LOWSPEED       9
#define USB_PORT_FEAT_HIGHSPEED      10
#define USB_PORT_FEAT_C_CONNECTION   16
#define USB_PORT_FEAT_C_ENABLE       17
#define USB_PORT_FEAT_C_SUSPEND      18
#define USB_PORT_FEAT_C_OVER_CURRENT 19
#define USB_PORT_FEAT_C_RESET        20
#define USB_PORT_FEAT_TEST           21

/*
 * Changes to Port feature numbers for Super speed,
 * from USB 3.0 spec Table 10-8
 */
#define USB_SS_PORT_FEAT_U1_TIMEOUT	23
#define USB_SS_PORT_FEAT_U2_TIMEOUT	24
#define USB_SS_PORT_FEAT_C_LINK_STATE	25
#define USB_SS_PORT_FEAT_C_CONFIG_ERROR	26
#define USB_SS_PORT_FEAT_BH_RESET	28
#define USB_SS_PORT_FEAT_C_BH_RESET	29

/* wPortStatus bits */
#define USB_PORT_STAT_CONNECTION    0x0001
#define USB_PORT_STAT_ENABLE        0x0002
#define USB_PORT_STAT_SUSPEND       0x0004
#define USB_PORT_STAT_OVERCURRENT   0x0008
#define USB_PORT_STAT_RESET         0x0010
#define USB_PORT_STAT_POWER         0x0100
#define USB_PORT_STAT_LOW_SPEED     0x0200
#define USB_PORT_STAT_HIGH_SPEED    0x0400	/* support for EHCI */
#define USB_PORT_STAT_SUPER_SPEED   0x0600	/* faking support to XHCI */
#define USB_PORT_STAT_SPEED_MASK	\
	(USB_PORT_STAT_LOW_SPEED | USB_PORT_STAT_HIGH_SPEED)

/*
 * Changes to wPortStatus bit field in USB 3.0
 * See USB 3.0 spec Table 10-10
 */
#define USB_SS_PORT_STAT_LINK_STATE	0x01e0
#define USB_SS_PORT_STAT_POWER		0x0200
#define USB_SS_PORT_STAT_SPEED		0x1c00
#define USB_SS_PORT_STAT_SPEED_5GBPS	0x0000
/* Bits that are the same from USB 2.0 */
#define USB_SS_PORT_STAT_MASK		(USB_PORT_STAT_CONNECTION | \
					 USB_PORT_STAT_ENABLE | \
					 USB_PORT_STAT_OVERCURRENT | \
					 USB_PORT_STAT_RESET)

/* wPortChange bits */
#define USB_PORT_STAT_C_CONNECTION  0x0001
#define USB_PORT_STAT_C_ENABLE      0x0002
#define USB_PORT_STAT_C_SUSPEND     0x0004
#define USB_PORT_STAT_C_OVERCURRENT 0x0008
#define USB_PORT_STAT_C_RESET       0x0010

/*
 * Changes to wPortChange bit fields in USB 3.0
 * See USB 3.0 spec Table 10-12
 */
#define USB_SS_PORT_STAT_C_BH_RESET	0x0020
#define USB_SS_PORT_STAT_C_LINK_STATE	0x0040
#define USB_SS_PORT_STAT_C_CONFIG_ERROR	0x0080

/* wHubCharacteristics (masks) */
#define HUB_CHAR_COMMON_OCPM        0x0000 /* All ports Over-Current reporting */
#define HUB_CHAR_INDV_PORT_LPSM     0x0001 /* per-port power control */
#define HUB_CHAR_NO_LPSM            0x0002 /* no power switching */
#define HUB_CHAR_LPSM               0x0003
#define HUB_CHAR_COMPOUND           0x0004
#define HUB_CHAR_INDV_PORT_OCPM     0x0008 /* per-port Over-current reporting */
#define HUB_CHAR_NO_OCPM            0x0010 /* No Over-current Protection support */
#define HUB_CHAR_OCPM               0x0018
#define HUB_CHAR_TTTT               0x0060 /* TT Think Time mask */
#define HUB_CHAR_PORTIND            0x0080 /* per-port indicators (LEDs) */

/*
 * Hub Status & Hub Change bit masks
 */
#define HUB_STATUS_LOCAL_POWER	0x0001
#define HUB_STATUS_OVERCURRENT	0x0002

#define HUB_CHANGE_LOCAL_POWER	0x0001
#define HUB_CHANGE_OVERCURRENT	0x0002

/* Mask for wIndex in get/set port feature */
#define USB_HUB_PORT_MASK	0xf

/* Hub class request codes */
#define USB_REQ_SET_HUB_DEPTH	0x0c

#endif /*_USB_DEFS_H_ */
