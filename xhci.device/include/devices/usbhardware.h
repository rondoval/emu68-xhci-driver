#ifndef DEVICES_USBHARDWARE_H
#define DEVICES_USBHARDWARE_H
/*
**	$VER: usbhardware.h 2.1 (14.02.07)
**
**	standard usb hardware device include file
**
**	(C) Copyright 2002-2007 Chris Hodges
**	    All Rights Reserved
*/

#ifndef EXEC_IO_H
#include "exec/io.h"
#endif

#ifndef EXEC_ERRORS_H
#include <exec/errors.h>
#endif

#ifndef DEVICES_USB_H
#include "devices/usb.h"
#endif

#if defined(__GNUC__)
# pragma pack(2)
#endif

/* IO Request structure */

struct IOUsbHWReq
{
    struct IORequest iouh_Req;
    UWORD iouh_Flags;          /* Transfer flags */
    UWORD iouh_State;          /* USB State Flags */
    UWORD iouh_Dir;            /* Direction of transfer */
    UWORD iouh_DevAddr;        /* USB Device Address (0-127) */
    UWORD iouh_Endpoint;       /* USB Device Endpoint (0-15) */
    UWORD iouh_MaxPktSize;     /* Maximum packet size for multiple packet transfers */
    ULONG iouh_Actual;         /* Actual bytes transferred */
    ULONG iouh_Length;         /* Size of buffer */
    APTR  iouh_Data;           /* Pointer to in/out buffer */
    UWORD iouh_Interval;       /* Interrupt Interval (in ms or 125 µSec units) */
    ULONG iouh_NakTimeout;     /* Timeout in ms before request will be retired */
    struct UsbSetupData iouh_SetupData; /* Setup fields for ctrl transfers */
    APTR  iouh_UserData;       /* private data, may not be touched by hardware driver,
                                  do not make assumptions about its contents */
    UWORD iouh_ExtError;       /* Extended error code */
    /* V2 structure extension */
    UWORD iouh_Frame;          /* current USB-Frame value */
    UWORD iouh_SplitHubAddr;   /* For Split-Transaction HUB address */
    UWORD iouh_SplitHubPort;   /* For Split-Transaction HUB downstream port */
    APTR  iouh_DriverPrivate1; /* private data for internal driver use */
    APTR  iouh_DriverPrivate2; /* private data for internal driver use */
};


struct IOUsbHWReqObsolete
{
    struct IORequest iouh_Req;
    UWORD iouh_Flags;          /* Transfer flags */
    UWORD iouh_State;          /* USB State Flags */
    UWORD iouh_Dir;            /* Direction of transfer */
    UWORD iouh_DevAddr;        /* USB Device Address (0-127) */
    UWORD iouh_Endpoint;       /* USB Device Endpoint (0-15) */
    UWORD iouh_MaxPktSize;     /* Maximum packet size for multiple packet transfers */
    ULONG iouh_Actual;         /* Actual bytes transferred */
    ULONG iouh_Length;         /* Size of buffer */
    APTR  iouh_Data;           /* Pointer to in/out buffer */
    UWORD iouh_Interval;       /* Interrupt Interval */
    ULONG iouh_NakTimeout;     /* Timeout in ms before request will be retired */
    struct UsbSetupData iouh_SetupData; /* Setup fields for ctrl transfers */
    APTR  iouh_UserData;       /* private data, may not be touched by hardware driver,
                                  do not make assumptions about its contents */
    UWORD iouh_ExtError;       /* Extended error code */
};

/* non-standard commands */

#define UHCMD_QUERYDEVICE (CMD_NONSTD+0)
#define UHCMD_USBRESET    (CMD_NONSTD+1)
#define UHCMD_USBRESUME   (CMD_NONSTD+2)
#define UHCMD_USBSUSPEND  CMD_STOP
#define UHCMD_USBOPER     CMD_START
#define UHCMD_CONTROLXFER (CMD_NONSTD+3)
#define UHCMD_ISOXFER     (CMD_NONSTD+4)
#define UHCMD_INTXFER     (CMD_NONSTD+5)
#define UHCMD_BULKXFER    (CMD_NONSTD+6)

/* Error codes for io_Error field */

#define UHIOERR_NO_ERROR      0   /* No error occured */
#define UHIOERR_USBOFFLINE    1   /* USB non-operational */
#define UHIOERR_NAK           2   /* NAK received */
#define UHIOERR_HOSTERROR     3   /* Unspecific host error */
#define UHIOERR_STALL         4   /* Endpoint stalled */
#define UHIOERR_PKTTOOLARGE   5   /* Packet is too large to be transferred */
#define UHIOERR_TIMEOUT       6   /* No acknoledge on packet */
#define UHIOERR_OVERFLOW      7   /* More data received than expected (babble condition) */
#define UHIOERR_CRCERROR      8   /* Incoming Packet corrupted */
#define UHIOERR_RUNTPACKET    9   /* Less data received than requested */
#define UHIOERR_NAKTIMEOUT   10   /* Timeout due to NAKs */
#define UHIOERR_BADPARAMS    11   /* Illegal parameters in request */
#define UHIOERR_OUTOFMEMORY  12   /* Out of auxiliary memory for the driver */

/* Values for iouh_Dir */

#define UHDIR_SETUP      0  /* This is a setup transfer (UHCMD_CTRLXFER) */
#define UHDIR_OUT        1  /* This is a host to device transfer */
#define UHDIR_IN         2  /* This is a device to host transfer */

/* Definitions for iouh_Flags */

#define UHFB_LOWSPEED      0  /* Device operates at low speed */
#define UHFB_HIGHSPEED     1  /* Device operates at high speed (USB 2.0) */
#define UHFB_NOSHORTPKT    2  /* Inhibit sending of a short packet at the end of a transfer (if possible) */
#define UHFB_NAKTIMEOUT    3  /* Allow the request to time-out after the given timeout value */
#define UHFB_ALLOWRUNTPKTS 4  /* Receiving less data than expected will not cause an UHIOERR_RUNTPACKET */
#define UHFB_SPLITTRANS    5  /* new for V2: Split transaction for Lowspeed/Fullspeed devices at USB2.0 hubs */
#define UHFB_MULTI_1       6  /* new for V2.1: Number of transactions per microframe bit 0 */
#define UHFB_MULTI_2       7  /* new for V2.1: Number of transactions per microframe bit 1 */

#define UHFF_LOWSPEED      (1<<UHFB_LOWSPEED)
#define UHFF_HIGHSPEED     (1<<UHFB_HIGHSPEED)
#define UHFF_NOSHORTPKT    (1<<UHFB_NOSHORTPKT)
#define UHFF_NAKTIMEOUT    (1<<UHFB_NAKTIMEOUT)
#define UHFF_ALLOWRUNTPKTS (1<<UHFB_ALLOWRUNTPKTS)
#define UHFF_SPLITTRANS    (1<<UHFB_SPLITTRANS)
#define UHFF_MULTI_1       (1<<UHFB_MULTI_1)
#define UHFF_MULTI_2       (1<<UHFB_MULTI_2)
#define UHFF_MULTI_3       ((1<<UHFB_MULTI_1)|(1<<UHFB_MULTI_2))

/* Tags for UHCMD_QUERYDEVICE */

#define UHA_Dummy          (TAG_USER  + 0x4711)
#define UHA_State          (UHA_Dummy + 0x01)
#define UHA_Manufacturer   (UHA_Dummy + 0x10)
#define UHA_ProductName    (UHA_Dummy + 0x11)
#define UHA_Version        (UHA_Dummy + 0x12)
#define UHA_Revision       (UHA_Dummy + 0x13)
#define UHA_Description    (UHA_Dummy + 0x14)
#define UHA_Copyright      (UHA_Dummy + 0x15)
#define UHA_DriverVersion  (UHA_Dummy + 0x20)

/* Definitions for UHA_State/iouh_State */

#define UHSB_OPERATIONAL 0 /* USB can be used for transfers */
#define UHSB_RESUMING    1 /* USB is currently resuming */
#define UHSB_SUSPENDED   2 /* USB is in suspended state */
#define UHSB_RESET       3 /* USB is just inside a reset phase */

#define UHSF_OPERATIONAL (1<<UHSB_OPERATIONAL)
#define UHSF_RESUMING    (1<<UHSB_RESUMING)
#define UHSF_SUSPENDED   (1<<UHSB_SUSPENDED)
#define UHSF_RESET       (1<<UHSB_RESET)

#if defined(__GNUC__)
# pragma pack()
#endif

#endif	/* DEVICES_USBHARDWARE_H */
