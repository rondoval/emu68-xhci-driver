#ifndef DEVICES_USB_H
#define DEVICES_USB_H
/*
**	$VER: usb.h 1.4 (16.08.02)
**
**	usb definitions include file
**
**	(C) Copyright 2002 Chris Hodges
**	    All Rights Reserved
*/

#if defined(__GNUC__)
# pragma pack(1)
#endif


/* Flags for bmRequestType */
#define URTF_OUT              0x00      /* direction: host to device */
#define URTF_IN               0x80      /* direction: device to host */
#define URTF_STANDARD         0x00      /* type: usb standard request */
#define URTF_CLASS            0x20      /* type: class request */
#define URTF_VENDOR           0x40      /* type: vendor specific request */
#define URTF_DEVICE           0x00      /* target: device */
#define URTF_INTERFACE        0x01      /* target: interface */
#define URTF_ENDPOINT         0x02      /* target: endpoint */
#define URTF_OTHER            0x03      /* target: other */

/* Usb Standard Requests (only for URTF_STANDARD) */
#define USR_GET_STATUS        0x00
#define USR_CLEAR_FEATURE     0x01
#define USR_GET_STATE         0x02 /* Hub */
#define USR_SET_FEATURE       0x03
#define USR_SET_ADDRESS       0x05
#define USR_GET_DESCRIPTOR    0x06
#define USR_SET_DESCRIPTOR    0x07
#define USR_GET_CONFIGURATION 0x08
#define USR_SET_CONFIGURATION 0x09
#define USR_GET_INTERFACE     0x0a
#define USR_SET_INTERFACE     0x0b
#define USR_SYNCH_FRAME       0x0c

/* Usb Hub Requests */
#define UHR_GET_STATE         0x02
#define UHR_CLEAR_TT_BUFFER   0x08
#define UHR_RESET_TT_BUFFER   0x09
#define UHR_GET_TT_STATE      0x0a
#define UHR_STOP_TT           0x0b

/* Usb Hid Requests */
#define UHR_GET_REPORT        0x01
#define UHR_GET_IDLE          0x02
#define UHR_GET_PROTOCOL      0x03
#define UHR_SET_REPORT        0x09
#define UHR_SET_IDLE          0x0a
#define UHR_SET_PROTOCOL      0x0b

/* Usb Printer Requests */
#define UPR_GET_DEVICE_ID     0x00
#define UPR_GET_PORT_STATUS   0x01
#define UPR_SOFT_RESET        0x02

/* Usb Mass Storage CBI Requests */
#define UMSR_ADSC             0x00

/* Usb Mass Storage Bulk only Requests */
#define UMSR_BULK_ONLY_RESET  0xff
#define UMSR_GET_MAX_LUN      0xfe

/* Usb CDC Requests (all of these target to an interface) */
#define UCDCR_SEND_ENCAPSULATED_COMMAND  0x00
#define UCDCR_GET_ENCAPSULATED_RESPONSE  0x01
#define UCDCR_SET_COMM_FEATURE           0x02
#define UCDCR_GET_COMM_FEATURE           0x03
#define UCDCR_CLEAR_COMM_FEATURE         0x04
#define UCDCR_SET_AUX_LINE_STATE         0x10
#define UCDCR_SET_HOOK_STATE             0x11
#define UCDCR_PULSE_SETUP                0x12
#define UCDCR_SEND_PULSE                 0x13
#define UCDCR_SET_PULSE_TIME             0x14
#define UCDCR_RING_AUX_JACK              0x15
#define UCDCR_SET_LINE_CODING            0x20
#define UCDCR_GET_LINE_CODING            0x21
#define UCDCR_SET_CONTROL_LINE_STATE     0x22
#define UCDCR_SEND_BREAK                 0x23
#define UCDCR_SET_RINGER_PARMS           0x30
#define UCDCR_GET_RINGER_PARMS           0x31
#define UCDCR_SET_OPERATION_PARMS        0x32
#define UCDCR_GET_OPERATION_PARMS        0x33
#define UCDCR_SET_LINE_PARMS             0x34
#define UCDCR_GET_LINE_PARMS             0x35
#define UCDCR_DIAL_DIGITS                0x36
#define UCDCR_SET_UNIT_PARAMETER         0x37
#define UCDCR_GET_UNIT_PARAMETER         0x38
#define UCDCR_CLEAR_UNIT_PARAMETER       0x39
#define UCDCR_GET_PROFILE                0x3A
#define UCDCR_SET_ETHERNET_MULTICAST_FILTERS 0x40
#define UCDCR_SET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER 0x41
#define UCDCR_GET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER 0x42
#define UCDCR_SET_ETHERNET_PACKET_FILTER 0x43
#define UCDCR_GET_ETHERNET_STATISTIC     0x44
#define UCDCR_SET_ATM_DATA_FORMAT        0x50
#define UCDCR_GET_ATM_DEVICE_STATISTICS  0x51
#define UCDCR_SET_ATM_DEFAULT_VC         0x52
#define UCDCR_GET_ATM_VC_STATISTICS      0x53

/* Usb CDC Notify Requests */
#define UCDCR_NETWORK_CONNECTION         0x00
#define UCDCR_RESPONSE_AVAILABLE         0x01
#define UCDCR_AUX_JACK_HOOK_STATE        0x08
#define UCDCR_RING_DETECT                0x09
#define UCDCR_SERIAL_STATE               0x20
#define UCDCR_CALL_STATE_CHANGE          0x28
#define UCDCR_LINE_STATE_CHANGE          0x29
#define UCDCR_CONNECTION_SPEED_CHANGE    0x2A

/* Usb DFU Requests */
#define UDFUR_DETACH              0x00
#define UDFUR_DNLOAD              0x01
#define UDFUR_UPLOAD              0x02
#define UDFUR_GETSTATUS           0x03
#define UDFUR_CLRSTATUS           0x04
#define UDFUR_GETSTATE            0x05
#define UDFUR_ABORT               0x06

/* Usb Standard Feature Selectors */
#define UFS_DEVICE_REMOTE_WAKEUP  0x01 /* Recipient: Device */
#define UFS_ENDPOINT_HALT         0x00 /* Recipient: Endpoint */

/* Usb Hub Feature Selectors */

#define UFS_C_HUB_LOCAL_POWER     0x00
#define UFS_C_HUB_OVER_CURRENT    0x01
#define UFS_PORT_CONNECTION       0x00
#define UFS_PORT_ENABLE           0x01
#define UFS_PORT_SUSPEND          0x02
#define UFS_PORT_OVER_CURRENT     0x03
#define UFS_PORT_RESET            0x04
#define UFS_PORT_POWER            0x08
#define UFS_PORT_LOW_SPEED        0x09
#define UFS_C_PORT_CONNECTION     0x10
#define UFS_C_PORT_ENABLE         0x11
#define UFS_C_PORT_SUSPEND        0x12
#define UFS_C_PORT_OVER_CURRENT   0x13
#define UFS_C_PORT_RESET          0x14
#define UFS_PORT_TEST             0x15
#define UFS_PORT_INDICATOR        0x16

/* Usb GetStatus() data bits (LE-UWORD) */
#define U_GSF_SELF_POWERED        8
#define U_GSF_REMOTE_WAKEUP       9

/* Usb Descriptor Types */
#define UDT_DEVICE            0x01
#define UDT_CONFIGURATION     0x02
#define UDT_STRING            0x03
#define UDT_INTERFACE         0x04
#define UDT_ENDPOINT          0x05

#define UDT_DEVICE_QUALIFIER  0x06
#define UDT_OTHERSPEED_QUALIFIER 0x07
#define UDT_INTERFACE_POWER   0x08
#define UDT_OTG               0x09
#define UDT_DEBUG             0x0a
#define UDT_INTERFACE_ASSOCIATION 0x0b
#define UDT_SECURITY          0x0c
#define UDT_ENCRYPTION_TYPE   0x0e
#define UDT_BOS               0x0f
#define UDT_DEVICE_CAPABILITY 0x10
#define UDT_WIRELESS_EP_COMP  0x11 /* Wireless endpoint companion descriptor */

/* HID class specific descriptors */
#define UDT_HID               0x21
#define UDT_REPORT            0x22
#define UDT_PHYSICAL          0x23

/* common class specific descriptors */

#define UDT_CS_UNDEFINED      0x20
#define UDT_CS_DEVICE         0x21
#define UDT_CS_CONFIGURATION  0x22
#define UDT_CS_STRING         0x23
#define UDT_CS_INTERFACE      0x24
#define UDT_CS_ENDPOINT       0x25

/* HUB class specific descriptors */
#define UDT_HUB               0x29

/* DFU class specific descriptors */
#define UDT_DFU               0x21

/* Device Wire Adapter class specific descriptors */
#define UDT_WIREADAPTER_CLASS 0x21 /* Wire Adapter class descriptor */
#define UDT_WIREADAPTER_RPIPE 0x22 /* Wire Adapter RPipe descriptor */
#define UDT_RADIO_CONTROL_IF  0x23 /* Radio Control Interface Class descriptor */

/* CDC WMC class specific descriptor subtypes */
#define UDST_CDC_HEADER       0x00 /* Header Functional Descriptor */
#define UDST_CDC_CALLMGM      0x01 /* Call Management Functional Descriptor */
#define UDST_CDC_ACM          0x02 /* Abstract Control Management Functional Descriptor */
#define UDST_CDC_DIRECTLINE   0x03 /* Direct Line Management Functional Descriptor */
#define UDST_CDC_PHONERING    0x04 /* Telephone Ringer Functional Descriptor */
#define UDST_CDC_PHONECALL    0x05 /* Telephone Call and Line State Reporting Capabilities Functional Descriptor */
#define UDST_CDC_UNION        0x06 /* Union Functional descriptor */
#define UDST_CDC_COUNTRY      0x07 /* Country Selection Functional Descriptor */
#define UDST_CDC_PHONEOP      0x08 /* Telephone Operational Modes Functional Descriptor */
#define UDST_CDC_TERMINAL     0x09 /* USB Terminal Functional Descriptor */
#define UDST_CDC_NETWORK      0x0a /* Network Channel Terminal Descriptor */
#define UDST_CDC_PROTOCOL     0x0b /* Protocol Unit Functional Descriptor */
#define UDST_CDC_EXTENSION    0x0c /* Extension Unit Functional Descriptor */
#define UDST_CDC_MCM          0x0d /* Multi-Channel Management Functional Descriptor */
#define UDST_CDC_CAPICTRL     0x0e /* CAPI Control Management Functional Descriptor */
#define UDST_CDC_ETHERNET     0x0f /* Ethernet Networking Functional Descriptor */
#define UDST_CDC_ATM          0x10 /* ATM Networking Functional Descriptor */
#define UDST_CDC_WMC          0x11 /* Wireless Handset Control Model Functional Descriptor */
#define UDST_CDC_MDLM         0x12 /* Mobile Direct Line Model Functional Descriptor */
#define UDST_CDC_MDLMDETAIL   0x13 /* MDLM Detail Functional Descriptor */
#define UDST_CDC_DEVMANGM     0x14 /* Device Management Model Functional Descriptor */
#define UDST_CDC_OBEX         0x15 /* OBEX Functional Descriptor */
#define UDST_CDC_CMDSET       0x16 /* Command Set Functional Descriptor */
#define UDST_CDC_CMDSETDETAIL 0x17 /* Command Set Detail Functional Descriptor */
#define UDST_CDC_TCM          0x18 /* Telephone Control Model Functional Descriptor */

/* Audio Ctrl class specific interface descriptor subtypes */
#define UDST_AUDIO_CTRL_HEADER          0x01
#define UDST_AUDIO_CTRL_INPUT_TERMINAL  0x02
#define UDST_AUDIO_CTRL_OUTPUT_TERMINAL 0x03
#define UDST_AUDIO_CTRL_MIXER_UNIT      0x04
#define UDST_AUDIO_CTRL_SELECTOR_UNIT   0x05
#define UDST_AUDIO_CTRL_FEATURE_UNIT    0x06
#define UDST_AUDIO_CTRL_PROCESSING_UNIT 0x07
#define UDST_AUDIO_CTRL_EXTENSION_UNIT  0x08

/* Audio Streaming class specific interface descriptor subtypes */
#define UDST_AUDIO_STREAM_GENERAL       0x01
#define UDST_AUDIO_STREAM_FMT_TYPE      0x02
#define UDST_AUDIO_STREAM_FMT_SPECIFIC  0x03

/* Audio MIDI class specific interface descriptor subtypes */
#define UDST_AUDIO_MIDI_HEADER   0x01
#define UDST_AUDIO_MIDI_IN_JACK  0x02
#define UDST_AUDIO_MIDI_OUT_JACK 0x03
#define UDST_AUDIO_MIDI_ELEMENT  0x04

/* Audioclass specific endpoint descriptors subtypes */
#define UDST_AUDIO_EP_GENERAL    0x01

/* Audio MIDI Jack-types */
#define USBMIDI_JACK_TYPE_UNDEFINED 0x00
#define USBMIDI_JACK_EMBEDDED       0x01
#define USBMIDI_JACK_EXTERNAL       0x02

/* Format of 8-bytes setup packet */

struct UsbSetupData
{
    UBYTE bmRequestType;       /* Request type and direction */
    UBYTE bRequest;            /* Request identifier */
    UWORD wValue;              /* request specific value, little endian! */
    UWORD wIndex;              /* request specific index, little endian! */
    UWORD wLength;             /* length of data to transfer, little endian! */
};

/* Usb Standard Device Descriptor */

struct UsbStdDevDesc
{
    UBYTE bLength;             /* Size of this descriptor in bytes */
    UBYTE bDescriptorType;     /* UDT_DEVICE Descriptor Type */
    UWORD bcdUSB;              /* USB Specification Release Number */
    UBYTE bDeviceClass;        /* Class code (assigned by the USB). */
    UBYTE bDeviceSubClass;     /* Subclass code (assigned by the USB). */
    UBYTE bDeviceProtocol;     /* Protocol code (assigned by the USB). */
    UBYTE bMaxPacketSize0;     /* Maximum packet size for endpoint zero (only 8, 16, 32, or 64 are valid) */
    UWORD idVendor;            /* Vendor ID (assigned by the USB) */
    UWORD idProduct;           /* Product ID (assigned by the manufacturer) */
    UWORD bcdDevice;           /* Device release number in binary-coded decimal */
    UBYTE iManufacturer;       /* Index of string descriptor describing manufacturer */
    UBYTE iProduct;            /* Index of string descriptor describing product */
    UBYTE iSerialNumber;       /* Index of string descriptor describing the device's serial number */
    UBYTE bNumConfigurations;  /* Number of possible configurations */
};

/* Usb Standard Configuration Descriptor */
struct UsbStdCfgDesc
{
    UBYTE bLength;             /* Size of this descriptor in bytes */
    UBYTE bDescriptorType;     /* UDT_CONFIGURATION Descriptor Type */
    UWORD wTotalLength;        /* Total length of data returned for this configuration. */
    UBYTE bNumInterfaces;      /* Number of interfaces supported by this configuration. */
    UBYTE bConfigurationValue; /* Value to use as an argument to the SetConfiguration() request */
    UBYTE iConfiguration;      /* Index of string descriptor describing this configuration */
    UBYTE bmAttributes;        /* Configuration characteristics */
    UBYTE bMaxPower;           /* Maximum power consumption of the USB device (2mA units) */
};

/* Flags for bmAttributes */
#define USCAF_ONE             0x80
#define USCAF_SELF_POWERED    0x40
#define USCAF_REMOTE_WAKEUP   0x20

struct UsbStdIfDesc
{
    UBYTE bLength;             /* Size of this descriptor in bytes */
    UBYTE bDescriptorType;     /* UDT_INTERFACE Descriptor Type */
    UBYTE bInterfaceNumber;    /* Number of interface. */
    UBYTE bAlternateSetting;   /* Value used to select alternate setting */
    UBYTE bNumEndpoints;       /* Number of endpoints used by this interface (excluding endpoint zero). */
    UBYTE bInterfaceClass;     /* Class code (assigned by the USB). */
    UBYTE bInterfaceSubClass;  /* Subclass code (assigned by the USB). */
    UBYTE bInterfaceProtocol;  /* Protocol code (assigned by the USB). */
    UBYTE iInterface;          /* Index of string descriptor describing this interface */
};

/* Usb Standard Endpoint Descriptor */

struct UsbStdEPDesc
{
    UBYTE bLength;             /* Size of this descriptor in bytes */
    UBYTE bDescriptorType;     /* UDT_ENDPOINT Descriptor Type */
    UBYTE bEndpointAddress;    /* The address of the endpoint on the USB, MSB holds direction */
    UBYTE bmAttributes;        /* TransferType (00=Control, 01=Iso, 10=Bulk, 11=Interrupt */
    UWORD wMaxPacketSize;      /* Maximum packet size this endpoint is capable of sending or receiving */
    UBYTE bInterval;           /* Interval for polling endpoint for data transfers in ms */
};

/* Usb Standard String Descriptors */
struct UsbStdStrDesc
{
    UBYTE bLength;             /* Size of this descriptor in bytes */
    UBYTE bDescriptorType;     /* UDT_STRING Descriptor Type */
    UWORD bString[1];          /* UNICODE encoded string */
};

/* Flags for bmAttributes */
#define USEAF_CONTROL         0x00
#define USEAF_ISOCHRONOUS     0x01
#define USEAF_BULK            0x02
#define USEAF_INTERRUPT       0x03

/* Usb Class Specific Descriptor: Hub Descriptor */
struct  UsbHubDesc
{
    UBYTE bLength;             /* Number of bytes in this descriptor, including this byte */
    UBYTE bDescriptorType;     /* Descriptor Type, value:  29H for hub descriptor */
    UBYTE bNbrPorts;           /* Number of downstream ports that this hub supports */
    UBYTE wHubCharacteristics; /* Hub flags (WORD!) (see below) */
    UBYTE wPad0;               /* Due to the wHubCharacteristics being on an ODD address, UWORD would add pad bytes! */
    UBYTE bPwrOn2PwrGood;      /* Time (in 2ms intervals) for power-good on port */
    UBYTE bHubContrCurrent;    /* Maximum current requirements of the Hub Controller in mA. */
    UBYTE DeviceRemovable;     /* Variable Size! Indicates if a port has a removable (0) device attached, Bit n<-> Port n */
    UBYTE PortPwrCtrlMask;     /* Variable Size! Obsolete (USB1.0) */
};

/* Flags for wHubCharacteristics */
#define UHCF_INDIVID_POWER    0x0100 /* Individual port power switching */
#define UHCF_IS_COMPOUND      0x0400 /* Hub is part of a compound device */
#define UHCF_INDIVID_OVP      0x0800 /* Individual port over-current status */
#define UHCF_NO_OVP           0x1000 /* No over-current protection */


/* Standard classes */
#define AUDIO_CLASSCODE       0x01
#define CDCCTRL_CLASSCODE     0x02
#define CDCDATA_CLASSCODE     0x0a
#define HID_CLASSCODE         0x03
#define PHYSICAL_CLASSCODE    0x05
#define STILLIMG_CLASSCODE    0x06
#define PRINTER_CLASSCODE     0x07
#define MASSSTORE_CLASSCODE   0x08
#define HUB_CLASSCODE         0x09
#define SMARTCARD_CLASSCODE   0x0b
#define SECURITY_CLASSCODE    0x0d
#define BLUETOOTH_CLASSCODE   0xe0
#define MISC_CLASSCODE        0xef
#define FWUPGRADE_CLASSCODE   0xfe
#define VENDOR_CLASSCODE      0xff

/* Audio classes */
#define AUDIO_NO_SUBCLASS     0x00
#define AUDIO_CTRL_SUBCLASS   0x01
#define AUDIO_STREAM_SUBCLASS 0x02
#define AUDIO_MIDI_SUBCLASS   0x03

/* Hid Subclasses */
#define HID_NO_SUBCLASS       0x00
#define HID_BOOT_SUBCLASS     0x01

/* Hid Proto if HID_BOOT_SUBCLASS */
#define HID_PROTO_KEYBOARD    0x01
#define HID_PROTO_MOUSE       0x02

/* Hid Proto values for UHR_SET_PROTOCOL */
#define HID_PROTO_BOOT        0x00
#define HID_PROTO_REPORT      0x01

/* Printer protocols */
#define PRT_PROTO_UNIDIR      0x01
#define PRT_PROTO_BIDIR       0x02
#define PRT_PROTO_IEEE1284    0x03

/* Mass Storage subclasses */
#define MS_RBC_SUBCLASS       0x01  /* Flash devices */
#define MS_ATAPI_SUBCLASS     0x02  /* CD roms etc */
#define MS_QIC157_SUBCLASS    0x03  /* Tape devices */
#define MS_UFI_SUBCLASS       0x04  /* Floppy */
#define MS_FDDATAPI_SUBCLASS  0x05  /* ATAPI Floppy */
#define MS_SCSI_SUBCLASS      0x06  /* SCSI devices */

/* Mass Storage protocols */
#define MS_PROTO_CBI          0x00 /* Control/Bulk/Interrupt transport
                                      with command completion interrupt */
#define MS_PROTO_CB           0x01 /* Control/Bulk/Interrupt transport
                                      without command completion interrupt */
#define MS_PROTO_BULK         0x50 /* Bulk-only transport */

/* Misc Subclasses */
#define MISC_COMMON_SUBCLASS  0x02

/* Misc Subclass Common protocols */
#define MISC_PROTO_WUSB_WAMP  0x02 /* Wire Adapter Multifunction Peripheral */

/* CDC Control Interface subclasses */
#define CDC_DLCM_SUBCLASS     0x01 /* Direct Line Control Model */
#define CDC_ACM_SUBCLASS      0x02 /* Abstract Control Model */
#define CDC_TCM_SUBCLASS      0x03 /* Telephone Control Model */
#define CDC_MCCM_SUBCLASS     0x04 /* Multi-Channel Control Model */
#define CDC_CAPICM_SUBCLASS   0x05 /* CAPI Control Model */
#define CDC_ETHCM_SUBCLASS    0x06 /* Ethernet Networking Control Model */
#define CDC_ATMCM_SUBCLASS    0x07 /* ATM Networking Control Model */
#define CDC_WHCM_SUBCLASS     0x08 /* Wireless Handset Control Model */
#define CDC_DEVMANGM_SUBCLASS 0x09 /* Device Management Model */
#define CDC_MDLM_SUBCLASS     0x0a /* Mobile Direct Line Model */
#define CDC_OBEX_SUBCLASS     0x0b /* OBEX Model */

/* CDC Control Interface protocols */
#define CDC_PROTO_USB         0x00 /* no class specific protocol */
#define CDC_PROTO_HAYES       0x01 /* common AT commands */
#define CDC_PROTO_PCCA101     0x02 /* AT Commands defined by [PCCA101] */
#define CDC_PROTO_PCCA101O    0x03 /* AT Commands defined by [PCCA101] + [PCCA101-O] */
#define CDC_PROTO_GSM0707     0x04 /* AT Commands defined by [GSM07.07] */
#define CDC_PROTO_3GPP27007   0x05 /* AT Commands defined by [3GPP27.007] */
#define CDC_PROTO_CS00170     0x06 /* AT Commands defined by [C-S0017-0] */
#define CDC_PROTO_EXTERNAL    0xfe /* External Protocol: Commands defined by Command Set functional descriptor */

/* CDC Data Interface protocols */
#define CDC_PROTO_ISDNBRI     0x30 /* I.430 Physical interface protocol for ISDN BRI */
#define CDC_PROTO_HDLC        0x31 /* ISO/IEC 3309-1993 HDLC */
#define CDC_PROTO_TRANSP      0x32 /* Transparent */
#define CDC_PROTO_Q921M       0x50 /* Management protocol for Q.921 data link protocol */
#define CDC_PROTO_Q921D       0x51 /* Data link protocol for Q.931 */
#define CDC_PROTO_Q921TM      0x52 /* TEI-multiplexor for Q.921 data link protocol */
#define CDC_PROTO_V42BIS      0x90 /* V.42bis Data compression procedures */
#define CDC_PROTO_EUROISDN    0x91 /* Q.931 Euro ISDN */
#define CDC_PROTO_V120        0x92 /* V.24 rate adaption to ISDN */
#define CDC_PROTO_CAPI20      0x93 /* CAPI Commands */
#define CDC_PROTO_HOST        0xFD /* host based driver */
#define CDC_PROTO_CDC         0xFE /* protocol described using protocol unit descriptors */

/* Bluetooth subclasses */
#define BLUETOOTH_RF_SUBCLASS 0x01
#define BLUETOOTH_WUSB_SUBCLASS 0x02 /* Wireless USB Wire Adapter */

/* Bluetooth RF protocols */
#define BLUETOOTH_PROTO_PRG   0x01
#define BLUETOOTH_PROTO_UWB   0x02 /* UWB Radio Control Interface Programming Interface */

/* Wireless WUSB protocols */
#define BLUETOOTH_PROTO_HWA   0x01 /* Host Wire Adapter Control */

/* Firmware Upgrade subclasses */
#define FWUPGRADE_STD_SUBCLASS 0x01

/* Firmware Upgrade protocols */
#define FWUPGRADE_PROTO_STD   0x01
#define FWUPGRADE_PROTO_DFU   0x02


/* Structure returned by GetHubStatus() */

struct UsbHubStatus
{
    UWORD wHubStatus;          /* Current status of hub (see below) */
    UWORD wHubChange;          /* Changes of status */
};

/* Flags for wHubStatus and wHubChange */
#define UHSF_LOCAL_POWER_LOST 0x0100
#define UHSF_OVER_CURRENT     0x0200

/* Structure returned by GetPortStatus() */
struct UsbPortStatus
{
    UWORD wPortStatus;         /* Current status of port (see below) */
    UWORD wPortChange;         /* Changes of status */
};

/* Flags for wPortStatus and wPortChange */
#define UPSF_PORT_CONNECTION  0x0100
#define UPSF_PORT_ENABLE      0x0200
#define UPSF_PORT_SUSPEND     0x0400
#define UPSF_PORT_OVER_CURRENT 0x0800
#define UPSF_PORT_RESET       0x1000
#define UPSF_PORT_POWER       0x0001
#define UPSF_PORT_LOW_SPEED   0x0002
#define UPSF_PORT_HIGH_SPEED  0x0004 /* USB 2.0 */
#define UPSF_PORT_TEST_MODE   0x0008 /* USB 2.0 */
#define UPSF_PORT_INDICATOR   0x0010 /* USB 2.0 */


/* Usb Class Specific Descriptor: HID Descriptor */

struct UsbHidDesc
{
    UBYTE bLength;             /* total size of the HID descriptor */
    UBYTE bDescriptorType;     /* Descriptor Type, value UDT_HID for HID */
    UWORD bcdHID;              /* the HID Class Spec release */
    UBYTE bCountryCode;        /* country code of the localized hardware. */
    UBYTE bNumDescriptors;     /* number of class descriptors (>=1) */
    UBYTE bDescType;           /* type of class descriptor */
    UBYTE wDescLength;         /* (WORD!) total size of the Report descriptor */
    UBYTE wPad0;
};

/* HID Report Item stuff */

#define REPORT_LONGITEM       0xfe
#define REPORT_ISIZE_0        0x00
#define REPORT_ISIZE_1        0x01
#define REPORT_ISIZE_2        0x02
#define REPORT_ISIZE_4        0x03
#define REPORT_ISIZE_MASK     0x03

#define REPORT_ITYPE_MAIN     0x00
#define REPORT_ITYPE_GLOBAL   0x04
#define REPORT_ITYPE_LOCAL    0x08
#define REPORT_ITYPE_MASK     0x0c

#define REPORT_ITAG_MASK      0xf0

/* main items */
#define REPORT_MAIN_INPUT     0x80
#define REPORT_MAIN_OUTPUT    0x90
#define REPORT_MAIN_COLLECT   0xa0
#define REPORT_MAIN_FEATURE   0xb0
#define REPORT_MAIN_ENDCOLL   0xc0

/* global items */
#define REPORT_GLOB_USAGE     0x00
#define REPORT_GLOB_LOGMIN    0x10
#define REPORT_GLOB_LOGMAX    0x20
#define REPORT_GLOB_PHYMIN    0x30
#define REPORT_GLOB_PHYMAX    0x40
#define REPORT_GLOB_UNITEXP   0x50
#define REPORT_GLOB_UNIT      0x60
#define REPORT_GLOB_RPSIZE    0x70
#define REPORT_GLOB_RPID      0x80
#define REPORT_GLOB_RPCOUNT   0x90
#define REPORT_GLOB_PUSH      0xa0
#define REPORT_GLOB_POP       0xb0

/* local items */
#define REPORT_LOCL_USAGE     0x00
#define REPORT_LOCL_USEMIN    0x10
#define REPORT_LOCL_USEMAX    0x20
#define REPORT_LOCL_DESIDX    0x30
#define REPORT_LOCL_DESMIN    0x40
#define REPORT_LOCL_DESMAX    0x50
#define REPORT_LOCL_STRIDX    0x70
#define REPORT_LOCL_STRMIN    0x80
#define REPORT_LOCL_STRMAX    0x90
#define REPORT_LOCL_DELIM     0xa0

/* Unit data definitons */

#define RP_UNIT_NONE          0x0

/* System units (nibble 0) */
#define RP_UNIT_SILINEAR      0x1
#define RP_UNIT_SIROTATION    0x2
#define RP_UNIT_ENGLINEAR     0x3
#define RP_UNIT ENGROTATION   0x4

/* Length units (nibble 1) */
#define RP_UNIT_CM            0x1
#define RP_UNIT_RAD           0x2
#define RP_UNIT_INCH          0x3
#define RP_UNIT_DEG           0x4

/* Mass units (nibble 2) */
#define RP_UNIT_GRAM1         0x1
#define RP_UNIT_GRAM2         0x2
#define RP_UNIT_SLUG1         0x3
#define RP_UNIT_SLUG2         0x4

/* Time units (nibble 3) */
#define RP_UNIT_SECS1         0x1
#define RP_UNIT_SECS2         0x2
#define RP_UNIT_SECS3         0x3
#define RP_UNIT_SECS4         0x4

/* Temperature units (nibble 4) */
#define RP_UNIT_KELVIN1       0x1
#define RP_UNIT_KELVIN2       0x2
#define RP_UNIT_FAHRENHEIT1   0x3
#define RP_UNIT_FAHRENHEIT2   0x4

/* Current units (nibble 5)   */
#define RP_UNIT_AMPERE1       0x1
#define RP_UNIT_AMPERE2       0x2
#define RP_UNIT_AMPERE3       0x3
#define RP_UNIT_AMPERE4       0x4

/* Lumious intensity units (nibble 6) */
#define RP_UNIT_CANDELA1      0x1
#define RP_UNIT_CANDELA2      0x2
#define RP_UNIT_CANDELA3      0x3
#define RP_UNIT_CANDELA4      0x4


/* Data for input/output/feature main items */

#define RPF_MAIN_CONST        0x0001 /* !DATA */
#define RPF_MAIN_VARIABLE     0x0002 /* !ARRAY */
#define RPF_MAIN_RELATIVE     0x0004 /* !ABSOLUTE */
#define RPF_MAIN_WRAP         0x0008 /* !NOWRAP */
#define RPF_MAIN_NONLINEAR    0x0010 /* !LINEAR */
#define RPF_MAIN_NOPREF       0x0020 /* !PREFERRED */
#define RPF_MAIN_NULLSTATE    0x0040 /* !NO NULL POS */
#define RPF_MAIN_VOLATILE     0x0080 /* !NON VOLATILE (only output/feature) */
#define RPF_MAIN_BUFBYTES     0x0100 /* !BITFIELD */

/* Data for collection main items */

#define RP_COLL_PHYSICAL      0x01 /* group of axes */
#define RP_COLL_APP           0x02 /* mouse, keyboard */
#define RP_COLL_LOGICAL       0x03 /* interrelated data */

/* Usage Page IDs */

#define RP_PAGE_GENERIC       0x01
#define RP_PAGE_SIMCTRLS      0x02
#define RP_PAGE_VRCTRLS       0x03
#define RP_PAGE_SPORTCTRLS    0x04
#define RP_PAGE_GAMECTRLS     0x05
#define RP_PAGE_KEYBOARD      0x07
#define RP_PAGE_LEDS          0x08
#define RP_PAGE_BUTTON        0x09
#define RP_PAGE_ORDINAL       0x0a
#define RP_PAGE_TELEPHONY     0x0b
#define RP_PAGE_CONSUMER      0x0c
#define RP_PAGE_DIGITIZER     0x0d
#define RP_PAGE_PID           0x0f
#define RP_PAGE_UNICODE       0x10
#define RP_PAGE_ALPHADISP     0x14
#define RP_PAGE_MONITOR       0x80
#define RP_PAGE_MONITORENUM   0x81
#define RP_PAGE_MONITORVESA   0x82
#define RP_PAGE_POWER         0x84 /* Verify this */
#define RP_PAGE_CAMERACTRL    0x90
#define RP_PAGE_ARCADE        0x91


/* Usb Mass Storage Class specific stuff */

struct UsbMSCmdBlkWrapper
{
    ULONG dCBWSignature;       /* 0x43425355 (little endian) indicating a CBW */
    ULONG dCBWTag;             /* Command Block Tag */
    ULONG dCBWDataTransferLength; /* Number of bytes to transfer */
    UBYTE bmCBWFlags;          /* Direction flag (Bit 7) */
    UBYTE bCBWLUN;             /* target logical unit number */
    UBYTE bCBWCBLength;        /* length of the command block in bytes (1-16) */
    UBYTE CBWCB[16];           /* the command block itself */
};

struct UsbMSCmdStatusWrapper
{
    ULONG dCSWSignature;       /* 0x53425355 (little endian) indicating a CSW */
    ULONG dCSWTag;             /* Command Status Tag, associated with dCBWTag */
    ULONG dCSWDataResidue;     /* Actual number of bytes to transfer */
    UBYTE bCSWStatus;          /* Status (see below) */
};

struct UsbMSCBIStatusWrapper
{
    UBYTE bType;               /* always 0x00 for valid status block (for UFI/Floppy, this is ASC) */
    UBYTE bValue;              /* mask out bit 0,1 for status (see below) (for UFI/Floppy, this is ASCQ) */
};

#define UMSCBW_SIZEOF         31 /* sizeof(UsbMSCmdBlkWrapper) will yield 32 instead of 31! */
#define UMSCSW_SIZEOF         13 /* sizeof(UsbMSCmdStatusWrapper) will yield 14 instead of 13! */

#define USMF_CSW_PASS         0x00 /* command passed */
#define USMF_CSW_FAIL         0x01 /* command failed */
#define USMF_CSW_PHASEERR     0x02 /* phase error */
#define USMF_CSW_PERSIST      0x03 /* persistant error (CBI only) */


/* Usb CDC Class specific stuff */

/* actual descriptors */

struct UsbCDCHeaderDesc
{
    UBYTE bFunctionLength;     /* total size of the descriptor */
    UBYTE bDescriptorType;     /* Descriptor Type */
    UBYTE bDescriptorSubtype;  /* Descriptor Subtype (CDC_FD_HEADER) */
    UBYTE bcdCDC0;             /* BCD Version lowbyte */
    UBYTE bcdCDC1;             /* BCD Version highbyte */
};

struct UsbCDCCallMgmDesc
{
    UBYTE bFunctionLength;     /* total size of the descriptor */
    UBYTE bDescriptorType;     /* Descriptor Type */
    UBYTE bDescriptorSubtype;  /* Descriptor Subtype (CDC_FD_CALLMGM) */
    UBYTE bmCapabilities;      /* Capabilities (bit 0 handles cm, bit 1 data interface) */
    UBYTE bDataInterface;      /* interface number for call management */
};

struct UsbCDCACMDesc
{
    UBYTE bFunctionLength;     /* total size of the descriptor */
    UBYTE bDescriptorType;     /* Descriptor Type */
    UBYTE bDescriptorSubtype;  /* Descriptor Subtype (CDC_FD_ACM) */
    UBYTE bmCapabilities;      /* Capabilities */
};

struct UsbCDCDLDesc
{
    UBYTE bFunctionLength;     /* total size of the descriptor */
    UBYTE bDescriptorType;     /* Descriptor Type */
    UBYTE bDescriptorSubtype;  /* Descriptor Subtype (CDC_FD_DirectLine) */
    UBYTE bmCapabilities;      /* Capabilities */
};

struct UsbCDCRingerDesc
{
    UBYTE bFunctionLength;     /* total size of the descriptor */
    UBYTE bDescriptorType;     /* Descriptor Type */
    UBYTE bDescriptorSubtype;  /* Descriptor Subtype (CDC_FD_PHONERING) */
    UBYTE bRingerVolSteps;     /* Volume scale */
    UBYTE bNumRingerPatterns;  /* Number of ring patterns supported */
};

struct UsbCDCPhoneOpDesc
{
    UBYTE bFunctionLength;     /* total size of the descriptor */
    UBYTE bDescriptorType;     /* Descriptor Type */
    UBYTE bDescriptorSubtype;  /* Descriptor Subtype (CDC_FD_PHONEOP) */
    UBYTE bmCapabilities;      /* Capabilities */
};

struct UsbCDCPhoneCallDesc
{
    UBYTE bFunctionLength;     /* total size of the descriptor */
    UBYTE bDescriptorType;     /* Descriptor Type */
    UBYTE bDescriptorSubtype;  /* Descriptor Subtype (CDC_FD_PHONECALL) */
    ULONG bmCapabilities;      /* Capabilities */
};

struct UsbCDCUnionDesc
{
    UBYTE bFunctionLength;     /* total size of the descriptor */
    UBYTE bDescriptorType;     /* Descriptor Type */
    UBYTE bDescriptorSubtype;  /* Descriptor Subtype (CDC_FD_UNION) */
    UBYTE bMasterInterface;    /* interface number of master interface */
    UBYTE bSlaveInterface0;    /* interface number of the first slave */
};

struct UsbCDCCountryDesc
{
    UBYTE bFunctionLength;     /* total size of the descriptor */
    UBYTE bDescriptorType;     /* Descriptor Type */
    UBYTE bDescriptorSubtype;  /* Descriptor Subtype (CDC_FD_COUNTRY) */
    UBYTE iCountryCodeRelDate; /* Index of a release date string */
    UWORD wCountryCode0;       /* Country code in hex */
};

struct UsbCDCTerminalDesc
{
    UBYTE bFunctionLength;     /* total size of the descriptor */
    UBYTE bDescriptorType;     /* Descriptor Type */
    UBYTE bDescriptorSubtype;  /* Descriptor Subtype (CDC_FD_TERMINAL) */
    UBYTE bEntityId;           /* constant identifying the terminal */
    UBYTE iName;               /* string index */
    UBYTE bChannelIndex;       /* channel index of associated network channel */
    UBYTE bPhysicalInterface;  /* type of physical interface (0=None, 1=ISDN) */
};

struct UsbCDCProtocolDesc
{
    UBYTE bFunctionLength;     /* total size of the descriptor */
    UBYTE bDescriptorType;     /* Descriptor Type */
    UBYTE bDescriptorSubtype;  /* Descriptor Subtype (CDC_FD_PROTOCOL) */
    UBYTE bEntityId;           /* constant identifying the unit */
    UBYTE bProtocol;           /* Protocol code */
    UBYTE bChildId0;           /* first ID of lower terminal or unit */
};

struct UsbCDCExtensionDesc
{
    UBYTE bFunctionLength;     /* total size of the descriptor */
    UBYTE bDescriptorType;     /* Descriptor Type */
    UBYTE bDescriptorSubtype;  /* Descriptor Subtype (CDC_FD_EXTENSION) */
    UBYTE bEntityId;           /* constant identifying the unit */
    UBYTE bExtensionCode;      /* vendor specific code */
    UBYTE iName;               /* string index */
    UBYTE bChildId0;           /* first ID of lower terminal or unit */
};

struct UsbCDCMCMDesc
{
    UBYTE bFunctionLength;     /* total size of the descriptor */
    UBYTE bDescriptorType;     /* Descriptor Type */
    UBYTE bDescriptorSubtype;  /* Descriptor Subtype (CDC_FD_MCM) */
    UBYTE bmCapabilities;      /* Capabilities */
};

struct UsbCDCCAPICtrlDesc
{
    UBYTE bFunctionLength;     /* total size of the descriptor */
    UBYTE bDescriptorType;     /* Descriptor Type */
    UBYTE bDescriptorSubtype;  /* Descriptor Subtype (CDC_FD_CAPICTRL) */
    UBYTE bmCapabilities;      /* Capabilities */
};

struct UsbCDCEthernetDesc
{
    UBYTE bFunctionLength;     /* total size of the descriptor */
    UBYTE bDescriptorType;     /* Descriptor Type */
    UBYTE bDescriptorSubtype;  /* Descriptor Subtype (CDC_FD_ETHERNET) */
    UBYTE iMACAddress;         /* index of string descriptor with MAC */
    ULONG bmEthernetStatistics; /* bitmap of statistics supported */
    UWORD wMaxSegmentSize;     /* MTU */
    UWORD wNumberMCFilters;    /* Number of multicast filters */
    UBYTE bNumberPowerFilters; /* Number of pattern filters for wake-up */
};

struct UsbCDCATMDesc
{
    UBYTE bFunctionLength;     /* total size of the descriptor */
    UBYTE bDescriptorType;     /* Descriptor Type */
    UBYTE bDescriptorSubtype;  /* Descriptor Subtype (CDC_FD_ATM) */
    UBYTE iEndSystemIdentifier; /* string index */
    UBYTE bmDataCapabilities;  /* Capabilities */
    UBYTE bmATMDeviceStatistics; /* bitmap of statistics supported */
    UWORD wType2MaxSegmentSize; /* MTU of Type 2 segments */
    UWORD wType3MaxSegmentSize; /* MTU of Type 3 segments */
    UWORD wMaxVC;              /* max number of virtual circuits */
};

/* CDC ACM specific stuff */

struct UsbCDCLineCoding
{
    ULONG dwDTERate;           /* Baud rate */
    UBYTE bCharFormat;         /* Stop bits 0->1, 1->1.5, 2->2 */
    UBYTE bParityType;         /* Party (0=None, 1=Odd, 2=Even, 3=Mark, 4=Space) */
    UBYTE bDataBits;           /* Databits (5,6,7,8 or 16) */
};

/* DFU specific descriptor */

struct UsbDFUDesc
{
    UBYTE bLength;             /* Size of this descriptor */
    UBYTE bDescriptorType;     /* Descriptor Type (0x21) */
    UBYTE bmAttributes;        /* DFU attributes */
    UBYTE wDetachTimeOut0;     /* Lowbyte of Detach TimeOut */
    UBYTE wDetachTimeOut1;     /* Highbyte of above */
    UBYTE wTransferSize0;      /* Maximum bytes per control-write (Lowbyte) */
    UBYTE wTransferSize1;      /* Highbyte of above */
    UBYTE bcdDFUVersion;       /* DFU Spec Release */
};

/* bmAttributes from above */
#define UDDAF_DOWNLOADABLE    0x0001
#define UDDAF_UPLOADABLE      0x0002
#define UDDAF_NO_MANIFEST_RST 0x0004
#define UDDAF_WILL_DETACH     0x0008

/* Returned by UDFUR_GETSTATUS */
struct UsbDFUStatus
{
    UBYTE bStatus;
    UBYTE bwPollTimeout0;
    UBYTE bwPollTimeout1;
    UBYTE bwPollTimeout2;
    UBYTE bState;
    UBYTE iString;
};

/* USB Audio Audio Data Formats */
#define UAADF_TYPE_I_UNDEFINED      0x0000
#define UAADF_PCM                   0x0001
#define UAADF_PCM8                  0x0002
#define UAADF_IEEE_FLOAT            0x0003
#define UAADF_ALAW                  0x0004
#define UAADF_MULAW                 0x0005

#define UAADF_TYPE_II_UNDEFINED     0x1000
#define UAADF_MPEG                  0x1001
#define UAADF_AC3                   0x1002

#define UAADF_TYPE_III_UNDEFINED    0x2000
#define UAADF_IEC1937_AC3           0x2001
#define UAADF_IEC1937_MPEG1_L1      0x2002
#define UAADF_IEC1937_MPEG1_L2_3    0x2003
#define UAADF_IEC1937_MPEG2_NOEXT   0x2003
#define UAADF_IEC1937_MPEG2_EXT     0x2004
#define UAADF_IEC1937_MPEG2_L1_LS   0x2005
#define UAADF_IEC1937_MPEG2_L2_3_LS 0x2006

/* USB Audio Format Type Codes */
#define UAFTC_UNDEFINED             0x00
#define UAFTC_TYPE_I                0x01
#define UAFTC_TYPE_II               0x02
#define UAFTC_TYPE_III              0x03

#if defined(__GNUC__)
# pragma pack()
#endif

#endif /* DEVICES_USB_H */
